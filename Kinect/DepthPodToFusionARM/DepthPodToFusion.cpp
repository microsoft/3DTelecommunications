#include "DepthPodToFusionAPI.h"
#include "DecodedCompressor.h"
#include "WorkerThreadT.h"
#include "TCPServer.h"
#include <boost/asio.hpp>
#include <vector>

struct FusionConnection
{
    StreamsDef streamDef;
    CWorkerThread<DecodedCompressor>* depthCompressionThread;
    TCPServer* server;
    boost::asio::io_service io_service_;
    TCPConnection* connection;
};


// static fusion connection list
static std::vector<FusionConnection*> s_connections;


long DepthToFusion_InitServer(
    unsigned int nStreams, 
    unsigned long depthSize, unsigned short bytesPerDepthPixel, 
    unsigned long colorSize, unsigned short bytesPerColorPixel,
    int& serverIdOut,
    unsigned short fusionPort, 
    unsigned short portOffset)
{
    serverIdOut = -1;

    FusionConnection* fc = new FusionConnection();
    fc->depthCompressionThread = nullptr;
    fc->server = nullptr;
    {
        //cout << "Starting fusion server at port " << (FUSION_PORT_BASE + PORT_OFFSET) << " ...";
        fc->server = new TCPServer(fc->io_service_, fusionPort + portOffset);
        fc->connection = new TCPConnection(fc->io_service_);
        fc->depthCompressionThread = new CWorkerThread<DecodedCompressor>();        
        fc->streamDef = 
        {
            nStreams,
            depthSize, 
            colorSize,
            bytesPerDepthPixel, 
            bytesPerColorPixel
        };
        fc->depthCompressionThread->InitStreams(fc->streamDef);

        fc->server->SetInput(static_cast<ICompressor*>(fc->depthCompressionThread));    
        HRESULT hr = fc->depthCompressionThread->Startup(1);
        if (FAILED(hr))
        {
            //cout << "ERROR starting compression thread." << endl;
            return hr;
        }
        
    }

    serverIdOut = (int)s_connections.size();
    s_connections.push_back(fc);

    return S_OK;
}


long DepthToFusion_DestroyServer(unsigned int serverId)
{
    if (serverId >= s_connections.size())
    {
        std::cout << "Invalid server ID.  Couldn't destroy." << std::endl;
        return E_INVALIDARG;
    }

    FusionConnection* fc = s_connections[serverId];
    if (fc == nullptr)
    {
        std::cout << "Invalid fusion connection.  Couldn't shutdown." << std::endl;
        return E_INVALIDARG;
    }

    std::cout << "Shutting down TCP connection..."; // this isn't finishing when I'm connected to fusion
    fc->connection->Shutdown();    
    std::cout << "Done.\nShutting down TCP Server...";
    fc->server->Shutdown();
    std::cout << "Done.\nShutting down compressor thread...";
    fc->depthCompressionThread->Shutdown();
    std::cout << "Done.\nClearing memory and returning." << std::endl;

    delete fc->depthCompressionThread;
    delete fc->server;
    delete fc->connection;
    delete fc;

    s_connections[serverId] = nullptr;

    return S_OK;
}


long DepthToFusion_SendFrame(
    unsigned int serverId, 
    unsigned char** depthFrames, 
    unsigned char** colorFrames, 
    unsigned char** audioFrames,
    std::vector<int> colorFrameSizes, 
    unsigned long frameNumber, 
    unsigned long timestamp)
{
    if (serverId < 0 ||
        serverId >= s_connections.size() ||
        s_connections[serverId] == nullptr)
    {
        return E_INVALIDARG;
    }

    FusionConnection* fc = s_connections[serverId];

    // Sending it into the compressor will compress it, and concatenate it on the end of the depth data.  It also automatically send it out the other end via Connection Manager	
    DWORD compressedBufferSize = 0;    

    // FYI uncompressed size = cTiledColorHeight * cTiledColorWidth * CPG_BYTES_PER_PIXEL + (header.mDepthHeight*header.mDepthWidth*header.mPodCount*TRANSMITTED_RAW_DEPTH_BYTES_PER_PIXEL);
    // where CPG_BYTES_PER_PIXEL = 1
    HRESULT hr = fc->depthCompressionThread->Compress(
        (USHORT**)depthFrames,
        (BYTE**)colorFrames,
        (USHORT**)audioFrames,
        frameNumber,
        timestamp,
        fc->streamDef.depthSize * fc->streamDef.bytesPerDepthPixel,
        colorFrameSizes,        
        compressedBufferSize);

    if (FAILED(hr))
    {
        //cout << "Failure compressing data.  Aborting." << endl;
        return hr;
    }

    return S_OK;
}
