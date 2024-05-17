///
/// Copyright (c) Microsoft Corporation. All Rights Reserved
///

#pragma once
#include <Synchapi.h>

template <class T> class CWorkerThread : public T
{
private:
    static const DWORD DEFAULT_POLL_TIME = 5;
    HANDLE m_hThread;
    HANDLE m_hStopEvent;
    DWORD m_dwPollTime;

public:
    float FramesPerSecond;

    CWorkerThread () :
		m_hThread(NULL),
		m_hStopEvent(NULL),
		m_dwPollTime(INFINITE),
        FramesPerSecond(0.0f)
	{
	}
	~CWorkerThread ()
	{
        if ( NULL != m_hStopEvent )
        {
			CloseHandle(m_hStopEvent);
		}
        if ( NULL != m_hThread )
        {
			CloseHandle(m_hThread);
		}
	}

    HRESULT Startup(DWORD dwPollTime = DEFAULT_POLL_TIME)
    {
        HRESULT hr;

        m_dwPollTime = dwPollTime;

        m_hStopEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
        if ( NULL == m_hStopEvent )
        {
            hr = HRESULT_FROM_WIN32(GetLastError());
            goto bailout;
        }

        hr = __super::Initialize();
        if ( FAILED(hr) )
        {
            goto bailout;
        }

        m_hThread = CreateThread(NULL, 0, _ThreadProc, this, 0, NULL);
        if ( NULL == m_hThread )
        {
            hr = HRESULT_FROM_WIN32(GetLastError());
            goto bailout;
        }

    bailout:
        return hr;
    }

	HRESULT Shutdown()
    {
        HRESULT hr = S_OK;

        if ( !SetEvent(m_hStopEvent) )
        {
            hr = HRESULT_FROM_WIN32(GetLastError());
            goto bailout;
        }

        if ( WAIT_OBJECT_0 != WaitForSingleObject(m_hThread, INFINITE) )
        {
            hr = HRESULT_FROM_WIN32(GetLastError());
            goto bailout;
        }

    bailout:
        hr = __super::Uninitialize();
        return hr;
    }

private:
    static DWORD WINAPI _ThreadProc(LPVOID pParam)
    {
        return ((CWorkerThread<T>*)pParam)->ThreadProc();
    }

    HRESULT ThreadProc()
    {
        HRESULT hr = S_OK;

        hr = ::CoInitializeEx(NULL, COINIT_MULTITHREADED);
        if ( FAILED(hr) )
        {
            goto bailout;
        }
		{
			LONGLONG llFrameCount = 0;
			LARGE_INTEGER liLastTick;
			QueryPerformanceCounter(&liLastTick);

			// Main thread loop
			while (SUCCEEDED(hr))
			{
				DWORD dwWaitResult = WaitForSingleObject(m_hStopEvent, m_dwPollTime);

				if (WAIT_OBJECT_0 == dwWaitResult)
				{
					break;
				}
				else
					if (WAIT_TIMEOUT == dwWaitResult)
					{
						hr = __super::Poll();

						if (S_OK == hr)
						{
							llFrameCount++;
						}

						LARGE_INTEGER liNewTick;
						LARGE_INTEGER liTickFreq;
						QueryPerformanceCounter(&liNewTick);
						QueryPerformanceFrequency(&liTickFreq);
						liNewTick.QuadPart -= liLastTick.QuadPart;
						liNewTick.QuadPart = (liNewTick.QuadPart * 1000000) / liTickFreq.QuadPart; // in microseconds
						if (liNewTick.QuadPart > 1000000)
						{
							// A second has elapsed, update FPS counter
							FramesPerSecond = (float)llFrameCount / (float)liNewTick.QuadPart * 1000000.0f;
							llFrameCount = 0;
							QueryPerformanceCounter(&liLastTick);
						}
						Sleep(0);
					}
					else
					{
						hr = HRESULT_FROM_WIN32(GetLastError());
					}
			}
		}
    bailout:
        CoUninitialize();
        return hr;
    }

};
