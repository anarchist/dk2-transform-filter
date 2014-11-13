#include <initguid.h>
#include <streams.h>
#include "DK2TransformFilter.h"

  STDMETHODIMP DK2TransformFilter::NonDelegatingQueryInterface(REFIID riid, void **ppv)
  {
    return CBaseFilter::NonDelegatingQueryInterface(riid,ppv);
  }


DK2TransformFilter::DK2TransformFilter(LPUNKNOWN pUnk, HRESULT *phr)
    : CTransformFilter(NAME("DK2 Transform Filter"), pUnk, CLSID_DK2TransformFilter)
  {
    /* Initialize any private variables here. */
  }

  HRESULT DK2TransformFilter::CheckInputType(const CMediaType *mtIn)
  {
    return S_OK;
  }

  HRESULT DK2TransformFilter::GetMediaType(int iPosition, CMediaType *pMediaType)
  {
    HRESULT hr = m_pInput->ConnectionMediaType(pMediaType);

    if (FAILED(hr))
      {
	return hr;
      }

    pMediaType->majortype = MEDIATYPE_Video;
    pMediaType->subtype = MEDIASUBTYPE_RGB24;
    pMediaType->SetTemporalCompression(FALSE);

    ASSERT(pMediaType->formattype == FORMAT_VideoInfo);

    VIDEOINFOHEADER *pVih =
      reinterpret_cast<VIDEOINFOHEADER*>(pMediaType->pbFormat);
    pVih->bmiHeader.biCompression = BI_RGB;
    pVih->bmiHeader.biBitCount = 24;
    pVih->bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    pVih->bmiHeader.biWidth = 752;
    pVih->bmiHeader.biHeight = 480;
    pVih->bmiHeader.biPlanes = 1;
    pVih->bmiHeader.biSizeImage = DIBSIZE(pVih->bmiHeader); 
    return S_OK;
  }

  HRESULT DK2TransformFilter::CheckTransform(const CMediaType *mtIn, const CMediaType *mtOut)
  {
    return S_OK;
  }

  HRESULT DK2TransformFilter::DecideBufferSize(IMemAllocator *pAlloc, ALLOCATOR_PROPERTIES *pProp)
  {
    AM_MEDIA_TYPE mt;
    HRESULT hr = m_pOutput->ConnectionMediaType(&mt);
    if (FAILED(hr))
      {
        return hr;
      }

    ASSERT(mt.formattype == FORMAT_VideoInfo);
    BITMAPINFOHEADER *pbmi = HEADER(mt.pbFormat);
    pProp->cbBuffer = DIBSIZE(*pbmi) * 2; 
    if (pProp->cbAlign == 0)
      {
        pProp->cbAlign = 1;
      }
    if (pProp->cBuffers == 0)
      {
        pProp->cBuffers = 1;
      }
    // Release the format block.
    FreeMediaType(mt);

    // Set allocator properties.
    ALLOCATOR_PROPERTIES Actual;
    hr = pAlloc->SetProperties(pProp, &Actual);
    if (FAILED(hr)) 
      {
        return hr;
      }
    // Even when it succeeds, check the actual result.
    if (pProp->cbBuffer > Actual.cbBuffer) 
      {
        return E_FAIL;
      }
    return S_OK;
  }

  DWORD DK2TransformFilter::EncodeFrame(BYTE* pBufferIn, BYTE* pBufferOut)
  {
    return 0;
  }

  HRESULT DK2TransformFilter::Transform(IMediaSample *pSource, IMediaSample *pDest)
  {
    // Get pointers to the underlying buffers.
    BYTE *pBufferIn, *pBufferOut;
    HRESULT hr = pSource->GetPointer(&pBufferIn);
    if (FAILED(hr))
      {
        return hr;
      }
    hr = pDest->GetPointer(&pBufferOut);
    if (FAILED(hr))
      {
        return hr;
      }
    // Process the data.
    DWORD cbDest = EncodeFrame(pBufferIn, pBufferOut);
    ASSERT((long)cbDest <= pDest->GetSize());

    pDest->SetActualDataLength(cbDest);
    pDest->SetSyncPoint(TRUE);
    return S_OK;
  }


  // COM/DLL registration boilerplate
  CUnknown * WINAPI DK2TransformFilter::CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr)
  {
    DK2TransformFilter *pFilter = new DK2TransformFilter(pUnk, pHr);
    if (pFilter== NULL) 
      {
        *pHr = E_OUTOFMEMORY;
      }
    return pFilter;
  }

  CFactoryTemplate g_Templates[] = 
    {
      { 
	L"DK2 Transform Filter",
	&CLSID_DK2TransformFilter,
	DK2TransformFilter::CreateInstance,
	NULL,
	NULL
      }
    };

int g_cTemplates = sizeof(g_Templates) / sizeof(g_Templates[0]);  


STDAPI DllRegisterServer()
{
	return AMovieDllRegisterServer2(TRUE);

} // DllRegisterServer


//
// DllUnregisterServer
//
STDAPI DllUnregisterServer()
{
	return AMovieDllRegisterServer2(FALSE);

} // DllUnregisterServer


//
// DllEntryPoint
//
extern "C" BOOL WINAPI DllEntryPoint(HINSTANCE, ULONG, LPVOID);

BOOL APIENTRY DllMain(HANDLE hModule,
	DWORD  dwReason,
	LPVOID lpReserved)
{
	return DllEntryPoint((HINSTANCE)(hModule), dwReason, lpReserved);
}
