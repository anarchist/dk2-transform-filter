#include <windows.h>
#include <initguid.h>
#include <streams.h>
#include "DK2TransformFilter.h"
#include "bayer.h"

STDMETHODIMP DK2TransformFilter::NonDelegatingQueryInterface(REFIID riid, void **ppv)
{
  return CBaseFilter::NonDelegatingQueryInterface(riid,ppv);
}

DK2TransformFilter::DK2TransformFilter(LPUNKNOWN pUnk, HRESULT *phr)
  : CTransformFilter(NAME("DK2 Transform Filter"), pUnk, CLSID_DK2TransformFilter)
{}

HRESULT DK2TransformFilter::CheckInputType(const CMediaType *mtIn)
{
  return S_OK;
}

HRESULT DK2TransformFilter::GetMediaType(int iPosition, CMediaType *pMediaType)
{
  if (m_pInput->IsConnected() == FALSE) {
    return E_UNEXPECTED;
  }

  if (iPosition > 0) {
    return VFW_S_NO_MORE_ITEMS;
  }

  pMediaType->SetType(&MEDIATYPE_Video);
  pMediaType->SetSubtype(&MEDIASUBTYPE_RGB24);
  pMediaType->SetTemporalCompression(FALSE);
  pMediaType->bFixedSizeSamples = true;
  pMediaType->bTemporalCompression = false;
  pMediaType->lSampleSize = 752 * 480 * 3;
  pMediaType->SetFormatType(&FORMAT_VideoInfo);
  ASSERT(pMediaType->formattype == FORMAT_VideoInfo);

  VIDEOINFO *pVih = (VIDEOINFO *)pMediaType->AllocFormatBuffer(sizeof(VIDEOINFO));
  ZeroMemory(pVih, sizeof(VIDEOINFO));

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
  pProp->cbBuffer = 752 * 480 * 3;
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
  long lSourceSize = pSource->GetActualDataLength();

#ifdef DEBUG
  long lDestSize = pDest->GetSize();
  ASSERT(lDestSize >= lSourceSize);
#endif

  dc1394_bayer_Bilinear(pBufferIn, pBufferOut, 752, 480, DC1394_COLOR_FILTER_RGGB);

  
  ASSERT((752 * 480 * 3) <= pDest->GetSize());

  pDest->SetActualDataLength(752 * 480 * 3);
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

const AMOVIESETUP_MEDIATYPE sudPinTypes =
  {
    &MEDIATYPE_Video,
    &MEDIASUBTYPE_YUY2
  };

const AMOVIESETUP_MEDIATYPE sudOutPinTypes = 
  {
    &MEDIATYPE_Video,
    &MEDIASUBTYPE_RGB24
  };

const AMOVIESETUP_PIN sudpPins[] =
  {
    { L"Input",
      FALSE,
      FALSE,
      FALSE,
      FALSE,
      &CLSID_NULL,
      NULL,
      1,
      &sudPinTypes
    },
    { L"Output",
      FALSE,
      TRUE,
      FALSE,
      FALSE,
      &CLSID_NULL,
      NULL,
      1,
      &sudOutPinTypes
    }
  };

const AMOVIESETUP_FILTER sudDK2 =
  {
    &CLSID_DK2TransformFilter,
    L"DK2 Transform Filter",
    MERIT_DO_NOT_USE,
    2,
    sudpPins
  };

CFactoryTemplate g_Templates[] = 
  {
    { 
      L"DK2 Transform Filter",
      &CLSID_DK2TransformFilter,
      DK2TransformFilter::CreateInstance,
      NULL,
      &sudDK2
    }
  };

int g_cTemplates = sizeof(g_Templates) / sizeof(g_Templates[0]);  

STDAPI DllRegisterServer()
{
  return AMovieDllRegisterServer2(TRUE);
}

STDAPI DllUnregisterServer()
{
  return AMovieDllRegisterServer2(FALSE);
}

extern "C" BOOL WINAPI DllEntryPoint(HINSTANCE, ULONG, LPVOID);

BOOL APIENTRY DllMain(HANDLE hModule,
		      DWORD  dwReason,
		      LPVOID lpReserved)
{
  return DllEntryPoint((HINSTANCE)(hModule), dwReason, lpReserved);
}
