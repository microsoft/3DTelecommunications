///
/// Copyright (c) Microsoft Corporation. All Rights Reserved
///

#include "../Win2Linux.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#pragma once


template <class T> class CWorkerThread : public T
{
private:
    typedef T __super;
    static const DWORD DEFAULT_POLL_TIME = 5;
    std::thread* m_hThread;
    std::mutex m_threadMutex;
    volatile bool m_hStopEvent;
    DWORD m_dwPollTime;

public:
    float FramesPerSecond;

    CWorkerThread () :
		m_hThread(NULL),
		m_hStopEvent(false),
		m_dwPollTime(INFINITE),
        FramesPerSecond(0.0f)
	{
	}
	virtual ~CWorkerThread ()
	{
        if ( NULL != m_hThread )
        {
            delete m_hThread;
		}
	}
    
    HRESULT Startup(DWORD dwPollTime = DEFAULT_POLL_TIME)
    {
        HRESULT hr;

        m_dwPollTime = dwPollTime;

        m_threadMutex.lock();
        {
            m_hStopEvent = false;
        }
        m_threadMutex.unlock();

        hr = __super::Initialize();
        if ( FAILED(hr) )
        {
            hr = -1;
            goto bailout;
        }

        try
        {
            m_hThread = new std::thread(_ThreadProc, this);
        }
        catch(const std::system_error& e)
        {
            std::cout << e.what() << std::endl;
            std::cout << e.code() << std::endl;
            hr = -1;
            goto bailout;
        }

    bailout:
        return hr;
    }

	HRESULT Shutdown()
    {
        HRESULT hr = S_OK;

        m_threadMutex.lock();
        {
            m_hStopEvent = true;
        }
        m_threadMutex.unlock();

        if(m_hThread == NULL)
        {
            printf("[WTHD] thread pointer was NULL\r\n");
            goto bailout;
        }
        else if(!m_hThread->joinable())
        {
            printf("[WTHD] thread was not joinable?\r\n");
            hr = -1;
            goto bailout;
        }
        else
        {
            m_hThread->join();
        }

    bailout:
        hr = __super::Uninitialize();
        return hr;
    }

private:
    static DWORD _ThreadProc(void* pParam)
    {
        return ((CWorkerThread<T>*)pParam)->ThreadProc();
    }

    HRESULT ThreadProc()
    {
        HRESULT hr = S_OK;

        if ( FAILED(hr) )
        {
            goto bailout;
        }
		{
			LONGLONG llFrameCount = 0;
			auto liLastTick = std::chrono::high_resolution_clock::now();

			// Main thread loop
			while (SUCCEEDED(hr))
			{           
                m_threadMutex.lock();
                {
                    if(m_hStopEvent)
                    {
                        goto bailout;
                    }
                }
                m_threadMutex.unlock();
        
                std::this_thread::sleep_for( std::chrono::seconds(m_dwPollTime) );
                {
                    hr = __super::Poll();

                    if (S_OK == hr)
                    {
                        llFrameCount++;
                    }

                    auto liNewTick = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double, std::milli> elapsed = liNewTick - liLastTick;
                    if (elapsed.count() > 1000)
                    {
                        // A second has elapsed, update FPS counter
                        FramesPerSecond = (float)llFrameCount / elapsed.count();
                        llFrameCount = 0;
                        liLastTick = std::chrono::high_resolution_clock::now();
                    }
                }
			}
		}
    bailout:
        //std::cout << "[WTHD] STOPPING" << std::endl;
        return hr;
    }
};
