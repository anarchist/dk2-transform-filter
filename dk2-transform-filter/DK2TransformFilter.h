#include <Amfilter.h>
#include <transfrm.h>


// {2B761529-21EC-4c1c-BDF5-0AAC8FC3EA0E}
DEFINE_GUID(CLSID_DK2TransformFilter,
	    0x2b761529, 0x21ec, 0x4c1c, 0xbd, 0xf5, 0xa, 0xac, 0x8f, 0xc3, 0xea, 0xe);


class DK2TransformFilter : public CTransformFilter {


 public:
  DECLARE_IUNKNOWN;
  STDMETHODIMP NonDelegatingQueryInterface(REFIID riid, void **ppv);
  static CUnknown * WINAPI CreateInstance(LPUNKNOWN punk, HRESULT *phr);

  // Overrriden from CTransformFilter base class
  HRESULT Transform(IMediaSample *pIn, IMediaSample *pOut);
  HRESULT CheckInputType(const CMediaType *mtIn);
  HRESULT CheckTransform(const CMediaType *mtIn, const CMediaType *mtOut);
  HRESULT DecideBufferSize(IMemAllocator *pAlloc,
			   ALLOCATOR_PROPERTIES *pProperties);
  HRESULT GetMediaType(int iPosition, CMediaType *pMediaType);
private:
  DK2TransformFilter(LPUNKNOWN punk, HRESULT *phr);
  DWORD DK2TransformFilter::EncodeFrame(BYTE* pBufferIn, BYTE* pBufferOut);
};
