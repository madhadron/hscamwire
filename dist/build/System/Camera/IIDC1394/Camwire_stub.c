#define IN_STG_CODE 0
#include "Rts.h"
#include "Stg.h"
#ifdef __cplusplus
extern "C" {
#endif
 
void SystemziCameraziIIDC1394ziCamwire_d1Kv(StgStablePtr the_stableptr, void* original_return_addr, HsPtr a1)
{
Capability *cap;
HaskellObj ret;
cap = rts_lock();
cap=rts_evalIO(cap,rts_apply(cap,(HaskellObj)runIO_closure,rts_apply(cap,(StgClosure*)deRefStablePtr(the_stableptr),rts_mkPtr(cap,a1))) ,&ret);
rts_checkSchedStatus("SystemziCameraziIIDC1394ziCamwire_d1Kv",cap);
rts_unlock(cap);
}
 
void SystemziCameraziIIDC1394ziCamwire_d1SJ(StgStablePtr the_stableptr, void* original_return_addr, HsPtr a1)
{
Capability *cap;
HaskellObj ret;
cap = rts_lock();
cap=rts_evalIO(cap,rts_apply(cap,(HaskellObj)runIO_closure,rts_apply(cap,(StgClosure*)deRefStablePtr(the_stableptr),rts_mkPtr(cap,a1))) ,&ret);
rts_checkSchedStatus("SystemziCameraziIIDC1394ziCamwire_d1SJ",cap);
rts_unlock(cap);
}
#ifdef __cplusplus
}
#endif

