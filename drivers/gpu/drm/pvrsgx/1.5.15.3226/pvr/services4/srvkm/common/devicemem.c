/*************************************************************************/ /*!
@Title          Device addressable memory functions
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Device addressable memory APIs
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/

#include <stddef.h>

#include "services_headers.h"
#include "buffer_manager.h"
#include "pdump_km.h"
#include "pvr_bridge_km.h"
#include "mm.h"

static PVRSRV_ERROR AllocDeviceMem(IMG_HANDLE		hDevCookie,
									IMG_HANDLE		hDevMemHeap,
									IMG_UINT32		ui32Flags,
									IMG_SIZE_T		ui32Size,
									IMG_SIZE_T		ui32Alignment,
									PVRSRV_KERNEL_MEM_INFO	**ppsMemInfo);

typedef struct _RESMAN_MAP_DEVICE_MEM_DATA_
{
	/* the DST meminfo created by the map */
	PVRSRV_KERNEL_MEM_INFO	*psMemInfo;
	/* SRC meminfo */
	PVRSRV_KERNEL_MEM_INFO	*psSrcMemInfo;
} RESMAN_MAP_DEVICE_MEM_DATA;


IMG_EXPORT
PVRSRV_ERROR IMG_CALLCONV PVRSRVGetDeviceMemHeapsKM(IMG_HANDLE hDevCookie,
													PVRSRV_HEAP_INFO *psHeapInfo)
{
	PVRSRV_DEVICE_NODE *psDeviceNode;
	IMG_UINT32 ui32HeapCount;
	DEVICE_MEMORY_HEAP_INFO *psDeviceMemoryHeap;
	IMG_UINT32 i;

	if (hDevCookie == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "PVRSRVGetDeviceMemHeapsKM: hDevCookie invalid"));
		PVR_DBG_BREAK;
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	psDeviceNode = (PVRSRV_DEVICE_NODE *)hDevCookie;

	/* Setup useful pointers */
	ui32HeapCount = psDeviceNode->sDevMemoryInfo.ui32HeapCount;
	psDeviceMemoryHeap = psDeviceNode->sDevMemoryInfo.psDeviceMemoryHeap;

	/* check we don't exceed the max number of heaps */
	PVR_ASSERT(ui32HeapCount <= PVRSRV_MAX_CLIENT_HEAPS);

	/* retrieve heap information */
	for(i=0; i<ui32HeapCount; i++)
	{
		/* return information about the heap */
		psHeapInfo[i].ui32HeapID = psDeviceMemoryHeap[i].ui32HeapID;
		psHeapInfo[i].hDevMemHeap = psDeviceMemoryHeap[i].hDevMemHeap;
		psHeapInfo[i].sDevVAddrBase = psDeviceMemoryHeap[i].sDevVAddrBase;
		psHeapInfo[i].ui32HeapByteSize = psDeviceMemoryHeap[i].ui32HeapSize;
		psHeapInfo[i].ui32Attribs = psDeviceMemoryHeap[i].ui32Attribs;
	}

	for(; i < PVRSRV_MAX_CLIENT_HEAPS; i++)
	{
		OSMemSet(psHeapInfo + i, 0, sizeof(*psHeapInfo));
		psHeapInfo[i].ui32HeapID = (IMG_UINT32)PVRSRV_UNDEFINED_HEAP_ID;
	}

	return PVRSRV_OK;
}

/*!
******************************************************************************

 @Function	PVRSRVCreateDeviceMemContextKM

 @Description

 Creates a device memory context

 @Input	   hDevCookie :
 @Input	   psPerProc : Per-process data
 @Output   phDevMemContext : ptr to handle to memory context
 @Output   pui32ClientHeapCount : ptr to heap count
 @Output   psHeapInfo : ptr to array of heap info

 @Return   PVRSRV_DEVICE_NODE, valid devnode or IMG_NULL

******************************************************************************/
IMG_EXPORT
PVRSRV_ERROR IMG_CALLCONV PVRSRVCreateDeviceMemContextKM(IMG_HANDLE					hDevCookie,
														 PVRSRV_PER_PROCESS_DATA	*psPerProc,
														 IMG_HANDLE 				*phDevMemContext,
														 IMG_UINT32 				*pui32ClientHeapCount,
														 PVRSRV_HEAP_INFO			*psHeapInfo,
														 IMG_BOOL					*pbCreated,
														 IMG_BOOL 					*pbShared)
{
	PVRSRV_DEVICE_NODE *psDeviceNode;
	IMG_UINT32 ui32HeapCount, ui32ClientHeapCount=0;
	DEVICE_MEMORY_HEAP_INFO *psDeviceMemoryHeap;
	IMG_HANDLE hDevMemContext;
	IMG_HANDLE hDevMemHeap;
	IMG_DEV_PHYADDR sPDDevPAddr;
	IMG_UINT32 i;

#if !defined(PVR_SECURE_HANDLES)
	PVR_UNREFERENCED_PARAMETER(pbShared);
#endif

	if (hDevCookie == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "PVRSRVCreateDeviceMemContextKM: hDevCookie invalid"));
		PVR_DBG_BREAK;
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	psDeviceNode = (PVRSRV_DEVICE_NODE *)hDevCookie;

	/*
		Setup useful pointers
	*/
	ui32HeapCount = psDeviceNode->sDevMemoryInfo.ui32HeapCount;
	psDeviceMemoryHeap = psDeviceNode->sDevMemoryInfo.psDeviceMemoryHeap;

	/*
		check we don't exceed the max number of heaps
	*/
	PVR_ASSERT(ui32HeapCount <= PVRSRV_MAX_CLIENT_HEAPS);

	/*
		Create a memory context for the caller
	*/
	hDevMemContext = BM_CreateContext(psDeviceNode,
									  &sPDDevPAddr,
									  psPerProc,
									  pbCreated);
	if (hDevMemContext == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVCreateDeviceMemContextKM: Failed BM_CreateContext"));
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	/* create the per context heaps */
	for(i=0; i<ui32HeapCount; i++)
	{
		switch(psDeviceMemoryHeap[i].DevMemHeapType)
		{
			case DEVICE_MEMORY_HEAP_SHARED_EXPORTED:
			{
				/* return information about the heap */
				psHeapInfo[ui32ClientHeapCount].ui32HeapID = psDeviceMemoryHeap[i].ui32HeapID;
				psHeapInfo[ui32ClientHeapCount].hDevMemHeap = psDeviceMemoryHeap[i].hDevMemHeap;
				psHeapInfo[ui32ClientHeapCount].sDevVAddrBase = psDeviceMemoryHeap[i].sDevVAddrBase;
				psHeapInfo[ui32ClientHeapCount].ui32HeapByteSize = psDeviceMemoryHeap[i].ui32HeapSize;
				psHeapInfo[ui32ClientHeapCount].ui32Attribs = psDeviceMemoryHeap[i].ui32Attribs;
#if defined(PVR_SECURE_HANDLES)
				pbShared[ui32ClientHeapCount] = IMG_TRUE;
#endif
				ui32ClientHeapCount++;
				break;
			}
			case DEVICE_MEMORY_HEAP_PERCONTEXT:
			{
				hDevMemHeap = BM_CreateHeap(hDevMemContext,
											&psDeviceMemoryHeap[i]);


				psHeapInfo[ui32ClientHeapCount].ui32HeapID = psDeviceMemoryHeap[i].ui32HeapID;
				psHeapInfo[ui32ClientHeapCount].hDevMemHeap = hDevMemHeap;
				psHeapInfo[ui32ClientHeapCount].sDevVAddrBase = psDeviceMemoryHeap[i].sDevVAddrBase;
				psHeapInfo[ui32ClientHeapCount].ui32HeapByteSize = psDeviceMemoryHeap[i].ui32HeapSize;
				psHeapInfo[ui32ClientHeapCount].ui32Attribs = psDeviceMemoryHeap[i].ui32Attribs;
#if defined(PVR_SECURE_HANDLES)
				pbShared[ui32ClientHeapCount] = IMG_FALSE;
#endif

				ui32ClientHeapCount++;
				break;
			}
		}
	}

	/* return shared_exported and per context heap information to the caller */
	*pui32ClientHeapCount = ui32ClientHeapCount;
	*phDevMemContext = hDevMemContext;

	return PVRSRV_OK;
}

IMG_EXPORT
PVRSRV_ERROR IMG_CALLCONV PVRSRVDestroyDeviceMemContextKM(IMG_HANDLE hDevCookie,
														  IMG_HANDLE hDevMemContext,
														  IMG_BOOL *pbDestroyed)
{
	PVR_UNREFERENCED_PARAMETER(hDevCookie);

	return BM_DestroyContext(hDevMemContext, pbDestroyed);
}




/*!
******************************************************************************

 @Function	PVRSRVGetDeviceMemHeapInfoKM

 @Description

 gets heap info

 @Input	   hDevCookie :
 @Input    hDevMemContext : ptr to handle to memory context
 @Output   pui32ClientHeapCount : ptr to heap count
 @Output   psHeapInfo : ptr to array of heap info

 @Return

******************************************************************************/
IMG_EXPORT
PVRSRV_ERROR IMG_CALLCONV PVRSRVGetDeviceMemHeapInfoKM(IMG_HANDLE					hDevCookie,
														 IMG_HANDLE 				hDevMemContext,
														 IMG_UINT32 				*pui32ClientHeapCount,
														 PVRSRV_HEAP_INFO			*psHeapInfo,
														 IMG_BOOL 					*pbShared)
{
	PVRSRV_DEVICE_NODE *psDeviceNode;
	IMG_UINT32 ui32HeapCount, ui32ClientHeapCount=0;
	DEVICE_MEMORY_HEAP_INFO *psDeviceMemoryHeap;
	IMG_HANDLE hDevMemHeap;
	IMG_UINT32 i;

#if !defined(PVR_SECURE_HANDLES)
	PVR_UNREFERENCED_PARAMETER(pbShared);
#endif

	if (hDevCookie == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "PVRSRVGetDeviceMemHeapInfoKM: hDevCookie invalid"));
		PVR_DBG_BREAK;
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	psDeviceNode = (PVRSRV_DEVICE_NODE *)hDevCookie;

	/*
		Setup useful pointers
	*/
	ui32HeapCount = psDeviceNode->sDevMemoryInfo.ui32HeapCount;
	psDeviceMemoryHeap = psDeviceNode->sDevMemoryInfo.psDeviceMemoryHeap;

	/*
		check we don't exceed the max number of heaps
	*/
	PVR_ASSERT(ui32HeapCount <= PVRSRV_MAX_CLIENT_HEAPS);

	/* create the per context heaps */
	for(i=0; i<ui32HeapCount; i++)
	{
		switch(psDeviceMemoryHeap[i].DevMemHeapType)
		{
			case DEVICE_MEMORY_HEAP_SHARED_EXPORTED:
			{

				psHeapInfo[ui32ClientHeapCount].ui32HeapID = psDeviceMemoryHeap[i].ui32HeapID;
				psHeapInfo[ui32ClientHeapCount].hDevMemHeap = psDeviceMemoryHeap[i].hDevMemHeap;
				psHeapInfo[ui32ClientHeapCount].sDevVAddrBase = psDeviceMemoryHeap[i].sDevVAddrBase;
				psHeapInfo[ui32ClientHeapCount].ui32HeapByteSize = psDeviceMemoryHeap[i].ui32HeapSize;
				psHeapInfo[ui32ClientHeapCount].ui32Attribs = psDeviceMemoryHeap[i].ui32Attribs;
#if defined(PVR_SECURE_HANDLES)
				pbShared[ui32ClientHeapCount] = IMG_TRUE;
#endif
				ui32ClientHeapCount++;
				break;
			}
			case DEVICE_MEMORY_HEAP_PERCONTEXT:
			{
				hDevMemHeap = BM_CreateHeap(hDevMemContext,
											&psDeviceMemoryHeap[i]);


				psHeapInfo[ui32ClientHeapCount].ui32HeapID = psDeviceMemoryHeap[i].ui32HeapID;
				psHeapInfo[ui32ClientHeapCount].hDevMemHeap = hDevMemHeap;
				psHeapInfo[ui32ClientHeapCount].sDevVAddrBase = psDeviceMemoryHeap[i].sDevVAddrBase;
				psHeapInfo[ui32ClientHeapCount].ui32HeapByteSize = psDeviceMemoryHeap[i].ui32HeapSize;
				psHeapInfo[ui32ClientHeapCount].ui32Attribs = psDeviceMemoryHeap[i].ui32Attribs;
#if defined(PVR_SECURE_HANDLES)
				pbShared[ui32ClientHeapCount] = IMG_FALSE;
#endif

				ui32ClientHeapCount++;
				break;
			}
		}
	}


	*pui32ClientHeapCount = ui32ClientHeapCount;

	return PVRSRV_OK;
}


/*!
******************************************************************************

 @Function	AllocDeviceMem

 @Description

 Allocates device memory

 @Input	   hDevCookie :

 @Input	   hDevMemHeap

 @Input	   ui32Flags : Some combination of PVRSRV_MEM_ flags

 @Input	   ui32Size :  Number of bytes to allocate

 @Input	   ui32Alignment : Alignment of allocation

 @Input    pvPrivData : Opaque private data passed through to allocator

 @Input    ui32PrivDataLength : Length of opaque private data

 @Output   **ppsMemInfo : On success, receives a pointer to the created MEM_INFO structure

 @Return   PVRSRV_ERROR :

******************************************************************************/
static PVRSRV_ERROR AllocDeviceMem(IMG_HANDLE		hDevCookie,
									IMG_HANDLE		hDevMemHeap,
									IMG_UINT32		ui32Flags,
									IMG_SIZE_T		ui32Size,
									IMG_SIZE_T		ui32Alignment,
									PVRSRV_KERNEL_MEM_INFO	**ppsMemInfo)
{
	PVRSRV_KERNEL_MEM_INFO	*psMemInfo;
	BM_HANDLE 		hBuffer;
	/* Pointer to implementation details within the mem_info */
	PVRSRV_MEMBLK	*psMemBlock;
	IMG_BOOL		bBMError;

	PVR_UNREFERENCED_PARAMETER(hDevCookie);

	*ppsMemInfo = IMG_NULL;

	if(OSAllocMem(PVRSRV_PAGEABLE_SELECT,
					sizeof(PVRSRV_KERNEL_MEM_INFO),
					(IMG_VOID **)&psMemInfo, IMG_NULL,
					"Kernel Memory Info") != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"AllocDeviceMem: Failed to alloc memory for block"));
		return (PVRSRV_ERROR_OUT_OF_MEMORY);
	}

	OSMemSet(psMemInfo, 0, sizeof(*psMemInfo));

	psMemBlock = &(psMemInfo->sMemBlk);

	/* BM supplied Device Virtual Address with physical backing RAM */
	psMemInfo->ui32Flags = ui32Flags | PVRSRV_MEM_RAM_BACKED_ALLOCATION;

	bBMError = BM_Alloc (hDevMemHeap,
							IMG_NULL,
							ui32Size,
							&psMemInfo->ui32Flags,
							IMG_CAST_TO_DEVVADDR_UINT(ui32Alignment),
							&hBuffer);

	if (!bBMError)
	{
		PVR_DPF((PVR_DBG_ERROR,"AllocDeviceMem: BM_Alloc Failed"));
		OSFreeMem(PVRSRV_PAGEABLE_SELECT, sizeof(PVRSRV_KERNEL_MEM_INFO), psMemInfo, IMG_NULL);
		/*not nulling pointer, out of scope*/
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	/* Fill in "Implementation dependant" section of mem info */
	psMemBlock->sDevVirtAddr = BM_HandleToDevVaddr(hBuffer);
	psMemBlock->hOSMemHandle = BM_HandleToOSMemHandle(hBuffer);

	/* Convert from BM_HANDLE to external IMG_HANDLE */
	psMemBlock->hBuffer = (IMG_HANDLE)hBuffer;

	/* Fill in the public fields of the MEM_INFO structure */

	psMemInfo->pvLinAddrKM = BM_HandleToCpuVaddr(hBuffer);

	psMemInfo->sDevVAddr = psMemBlock->sDevVirtAddr;

	psMemInfo->ui32AllocSize = ui32Size;


	psMemInfo->pvSysBackupBuffer = IMG_NULL;


	*ppsMemInfo = psMemInfo;


	return (PVRSRV_OK);
}

static PVRSRV_ERROR FreeDeviceMem2(PVRSRV_KERNEL_MEM_INFO *psMemInfo, IMG_BOOL bFromAllocator)
{
	BM_HANDLE		hBuffer;

	if (!psMemInfo)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	hBuffer = psMemInfo->sMemBlk.hBuffer;


	if (bFromAllocator)
		BM_Free(hBuffer, psMemInfo->ui32Flags);
	else
		BM_FreeExport(hBuffer, psMemInfo->ui32Flags);


	if ((psMemInfo->pvSysBackupBuffer) && bFromAllocator)
	{

		OSFreeMem(PVRSRV_OS_PAGEABLE_HEAP, psMemInfo->ui32AllocSize, psMemInfo->pvSysBackupBuffer, IMG_NULL);
		psMemInfo->pvSysBackupBuffer = IMG_NULL;
	}

	if (psMemInfo->ui32RefCount == 0)
		OSFreeMem(PVRSRV_PAGEABLE_SELECT, sizeof(PVRSRV_KERNEL_MEM_INFO), psMemInfo, IMG_NULL);


	return(PVRSRV_OK);
}

static PVRSRV_ERROR FreeDeviceMem(PVRSRV_KERNEL_MEM_INFO *psMemInfo)
{
	BM_HANDLE		hBuffer;

	if (!psMemInfo)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	hBuffer = psMemInfo->sMemBlk.hBuffer;


	BM_Free(hBuffer, psMemInfo->ui32Flags);

	if(psMemInfo->pvSysBackupBuffer)
	{

		OSFreeMem(PVRSRV_OS_PAGEABLE_HEAP, psMemInfo->ui32AllocSize, psMemInfo->pvSysBackupBuffer, IMG_NULL);
		psMemInfo->pvSysBackupBuffer = IMG_NULL;
	}

	OSFreeMem(PVRSRV_PAGEABLE_SELECT, sizeof(PVRSRV_KERNEL_MEM_INFO), psMemInfo, IMG_NULL);


	return(PVRSRV_OK);
}


/*!
******************************************************************************

 @Function	PVRSRVAllocSyncInfoKM

 @Description

 Allocates a sync info

 @Return   PVRSRV_ERROR :

******************************************************************************/
IMG_EXPORT
PVRSRV_ERROR IMG_CALLCONV PVRSRVAllocSyncInfoKM(IMG_HANDLE					hDevCookie,
												IMG_HANDLE					hDevMemContext,
												PVRSRV_KERNEL_SYNC_INFO		**ppsKernelSyncInfo)
{
	IMG_HANDLE hSyncDevMemHeap;
	DEVICE_MEMORY_INFO *psDevMemoryInfo;
	BM_CONTEXT *pBMContext;
	PVRSRV_ERROR eError;
	PVRSRV_KERNEL_SYNC_INFO	*psKernelSyncInfo;
	PVRSRV_SYNC_DATA *psSyncData;

	eError = OSAllocMem(PVRSRV_PAGEABLE_SELECT,
						sizeof(PVRSRV_KERNEL_SYNC_INFO),
						(IMG_VOID **)&psKernelSyncInfo, IMG_NULL,
						"Kernel Synchronization Info");
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVAllocSyncInfoKM: Failed to alloc memory"));
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	psKernelSyncInfo->ui32RefCount = 0;

	/* Get the devnode from the devheap */
	pBMContext = (BM_CONTEXT*)hDevMemContext;
	psDevMemoryInfo = &pBMContext->psDeviceNode->sDevMemoryInfo;

	/* and choose a heap for the syncinfo */
	hSyncDevMemHeap = psDevMemoryInfo->psDeviceMemoryHeap[psDevMemoryInfo->ui32SyncHeapID].hDevMemHeap;

	/*
		Cache consistent flag would be unnecessary if the heap attributes were
		changed to specify it.
	*/
	eError = AllocDeviceMem(hDevCookie,
							hSyncDevMemHeap,
							PVRSRV_MEM_CACHE_CONSISTENT,
							sizeof(PVRSRV_SYNC_DATA),
							sizeof(IMG_UINT32),
							&psKernelSyncInfo->psSyncDataMemInfoKM);

	if (eError != PVRSRV_OK)
	{

		PVR_DPF((PVR_DBG_ERROR,"PVRSRVAllocSyncInfoKM: Failed to alloc memory"));
		OSFreeMem(PVRSRV_PAGEABLE_SELECT, sizeof(PVRSRV_KERNEL_SYNC_INFO), psKernelSyncInfo, IMG_NULL);
		/*not nulling pointer, out of scope*/
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	/* init sync data */
	psKernelSyncInfo->psSyncData = psKernelSyncInfo->psSyncDataMemInfoKM->pvLinAddrKM;
	psSyncData = psKernelSyncInfo->psSyncData;

	psSyncData->ui32WriteOpsPending = 0;
	psSyncData->ui32WriteOpsComplete = 0;
	psSyncData->ui32ReadOpsPending = 0;
	psSyncData->ui32ReadOpsComplete = 0;
	psSyncData->ui32LastOpDumpVal = 0;
	psSyncData->ui32LastReadOpDumpVal = 0;

#if defined(PDUMP)
	PDUMPMEM(psKernelSyncInfo->psSyncDataMemInfoKM->pvLinAddrKM,
			psKernelSyncInfo->psSyncDataMemInfoKM,
			0,
			psKernelSyncInfo->psSyncDataMemInfoKM->ui32AllocSize,
			PDUMP_FLAGS_CONTINUOUS,
			MAKEUNIQUETAG(psKernelSyncInfo->psSyncDataMemInfoKM));
#endif

	psKernelSyncInfo->sWriteOpsCompleteDevVAddr.uiAddr = psKernelSyncInfo->psSyncDataMemInfoKM->sDevVAddr.uiAddr + offsetof(PVRSRV_SYNC_DATA, ui32WriteOpsComplete);
	psKernelSyncInfo->sReadOpsCompleteDevVAddr.uiAddr = psKernelSyncInfo->psSyncDataMemInfoKM->sDevVAddr.uiAddr + offsetof(PVRSRV_SYNC_DATA, ui32ReadOpsComplete);

	/* syncinfo meminfo has no syncinfo! */
	psKernelSyncInfo->psSyncDataMemInfoKM->psKernelSyncInfo = IMG_NULL;


	/* return result */
	*ppsKernelSyncInfo = psKernelSyncInfo;

	return PVRSRV_OK;
}


IMG_EXPORT
PVRSRV_ERROR IMG_CALLCONV PVRSRVFreeSyncInfoKM(PVRSRV_KERNEL_SYNC_INFO	*psKernelSyncInfo)
{
	PVRSRV_ERROR eError;

	if (psKernelSyncInfo->ui32RefCount != 0)
	{
		PVR_DPF((PVR_DBG_ERROR, "oops: sync info ref count not zero at destruction"));

		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	eError = FreeDeviceMem(psKernelSyncInfo->psSyncDataMemInfoKM);
	(IMG_VOID)OSFreeMem(PVRSRV_PAGEABLE_SELECT, sizeof(PVRSRV_KERNEL_SYNC_INFO), psKernelSyncInfo, IMG_NULL);


	return eError;
}

static IMG_VOID freeWrapped(PVRSRV_KERNEL_MEM_INFO *psMemInfo)
{
	IMG_HANDLE hOSWrapMem = psMemInfo->sMemBlk.hOSWrapMem;

	/* free the page addr array if req'd */
	if(psMemInfo->sMemBlk.psIntSysPAddr)
	{
		OSFreeMem(PVRSRV_OS_PAGEABLE_HEAP, sizeof(IMG_SYS_PHYADDR), psMemInfo->sMemBlk.psIntSysPAddr, IMG_NULL);
		psMemInfo->sMemBlk.psIntSysPAddr = IMG_NULL;
	}

	if(hOSWrapMem)
	{
		OSReleasePhysPageAddr(hOSWrapMem);
	}
}

static PVRSRV_ERROR FreeMemCallBackCommon(PVRSRV_KERNEL_MEM_INFO *psMemInfo,
										  IMG_UINT32	ui32Param,
										  IMG_BOOL		bFromAllocator)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	PVR_UNREFERENCED_PARAMETER(ui32Param);


	psMemInfo->ui32RefCount--;


	if((psMemInfo->ui32Flags & PVRSRV_MEM_EXPORTED) && (bFromAllocator == IMG_TRUE))
	{
		IMG_HANDLE hMemInfo = IMG_NULL;


		eError = PVRSRVFindHandle(KERNEL_HANDLE_BASE,
								 &hMemInfo,
								 psMemInfo,
								 PVRSRV_HANDLE_TYPE_MEM_INFO);
		if(eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "FreeMemCallBackCommon: can't find exported meminfo in the global handle list"));
			return eError;
		}


		eError = PVRSRVReleaseHandle(KERNEL_HANDLE_BASE,
									hMemInfo,
									PVRSRV_HANDLE_TYPE_MEM_INFO);
		if(eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "FreeMemCallBackCommon: PVRSRVReleaseHandle failed for exported meminfo"));
			return eError;
		}
	}


	if (psMemInfo->ui32RefCount == 0)
	{
		switch(psMemInfo->memType)
		{

			case PVRSRV_MEMTYPE_WRAPPED:
				freeWrapped(psMemInfo);
			case PVRSRV_MEMTYPE_DEVICE:
				if (psMemInfo->psKernelSyncInfo)
				{
					psMemInfo->psKernelSyncInfo->ui32RefCount--;

					if (psMemInfo->psKernelSyncInfo->ui32RefCount == 0)
					{
						eError = PVRSRVFreeSyncInfoKM(psMemInfo->psKernelSyncInfo);
					}
				}
			case PVRSRV_MEMTYPE_DEVICECLASS:
				break;
			default:
				PVR_DPF((PVR_DBG_ERROR, "FreeMemCallBackCommon: Unknown memType"));
				return PVRSRV_ERROR_GENERIC;
		}
	}

	/*
	 * FreeDeviceMem2 will do the right thing, freeing
	 * the virtual memory info when the allocator calls
	 * but only releaseing the physical pages when everyone
	 * is done.
	 */

	return FreeDeviceMem2(psMemInfo, bFromAllocator);
}

static PVRSRV_ERROR FreeDeviceMemCallBack(IMG_PVOID pvParam,
										  IMG_UINT32 ui32Param)
{
	PVRSRV_KERNEL_MEM_INFO *psMemInfo = (PVRSRV_KERNEL_MEM_INFO *)pvParam;

	return FreeMemCallBackCommon(psMemInfo, ui32Param, IMG_TRUE);
}


/*!
******************************************************************************

 @Function	PVRSRVFreeDeviceMemKM

 @Description

 Frees memory allocated with PVRAllocDeviceMem, including the mem_info structure

 @Input	   psMemInfo :

 @Return   PVRSRV_ERROR  :

******************************************************************************/
IMG_EXPORT
PVRSRV_ERROR IMG_CALLCONV PVRSRVFreeDeviceMemKM(IMG_HANDLE				hDevCookie,
												PVRSRV_KERNEL_MEM_INFO	*psMemInfo)
{
	PVRSRV_ERROR eError;

	PVR_UNREFERENCED_PARAMETER(hDevCookie);

	if (!psMemInfo)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	if (psMemInfo->sMemBlk.hResItem != IMG_NULL)
	{
		eError = ResManFreeResByPtr(psMemInfo->sMemBlk.hResItem);
	}
	else
	{

		eError = FreeDeviceMemCallBack(psMemInfo, 0);
	}

	return eError;
}


/*!
******************************************************************************

 @Function	PVRSRVRemapToDevKM

 @Description

 Remaps buffer to GPU virtual address space

 @Input    psMemInfo

 @Return   PVRSRV_ERROR :

******************************************************************************/
IMG_EXPORT
PVRSRV_ERROR IMG_CALLCONV _PVRSRVAllocDeviceMemKM(IMG_HANDLE					hDevCookie,
												 PVRSRV_PER_PROCESS_DATA	*psPerProc,
												 IMG_HANDLE					hDevMemHeap,
												 IMG_UINT32					ui32Flags,
												 IMG_SIZE_T					ui32Size,
												 IMG_SIZE_T					ui32Alignment,
												 PVRSRV_KERNEL_MEM_INFO		**ppsMemInfo)
{
	PVRSRV_KERNEL_MEM_INFO	*psMemInfo;
	PVRSRV_ERROR 			eError;
	BM_HEAP					*psBMHeap;
	IMG_HANDLE				hDevMemContext;

	if (!hDevMemHeap ||
		(ui32Size == 0))
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}


	if (ui32Flags & PVRSRV_HAP_CACHETYPE_MASK)
	{
		if (((ui32Size % HOST_PAGESIZE()) != 0) ||
			((ui32Alignment % HOST_PAGESIZE()) != 0))
		{
			return PVRSRV_ERROR_INVALID_PARAMS;
		}
	}

	eError = AllocDeviceMem(hDevCookie,
							hDevMemHeap,
							ui32Flags,
							ui32Size,
							ui32Alignment,
							&psMemInfo);

	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	if (ui32Flags & PVRSRV_MEM_NO_SYNCOBJ)
	{
		psMemInfo->psKernelSyncInfo = IMG_NULL;
	}
	else
	{
		/*
			allocate a syncinfo but don't register with resman
			because the holding devicemem will handle the syncinfo
		*/
		psBMHeap = (BM_HEAP*)hDevMemHeap;
		hDevMemContext = (IMG_HANDLE)psBMHeap->pBMContext;
		eError = PVRSRVAllocSyncInfoKM(hDevCookie,
									   hDevMemContext,
									   &psMemInfo->psKernelSyncInfo);
		if(eError != PVRSRV_OK)
		{
			goto free_mainalloc;
		}
		psMemInfo->psKernelSyncInfo->ui32RefCount++;
	}

	/*
	 * Setup the output.
	 */
	*ppsMemInfo = psMemInfo;

	if (ui32Flags & PVRSRV_MEM_NO_RESMAN)
	{
		psMemInfo->sMemBlk.hResItem = IMG_NULL;
	}
	else
	{
		/* register with the resman */
		psMemInfo->sMemBlk.hResItem = ResManRegisterRes(psPerProc->hResManContext,
														RESMAN_TYPE_DEVICEMEM_ALLOCATION,
														psMemInfo,
														0,
														FreeDeviceMemCallBack);
		if (psMemInfo->sMemBlk.hResItem == IMG_NULL)
		{

			eError = PVRSRV_ERROR_OUT_OF_MEMORY;
			goto free_mainalloc;
		}
	}


	psMemInfo->ui32RefCount++;

	psMemInfo->memType = PVRSRV_MEMTYPE_DEVICE;

	/*
	 * And I think we're done for now....
	 */
	return (PVRSRV_OK);

free_mainalloc:
	FreeDeviceMem(psMemInfo);

	return eError;
}


IMG_EXPORT
PVRSRV_ERROR IMG_CALLCONV PVRSRVDissociateDeviceMemKM(IMG_HANDLE              hDevCookie,
													  PVRSRV_KERNEL_MEM_INFO *psMemInfo)
{
	PVRSRV_ERROR		eError;
	PVRSRV_DEVICE_NODE	*psDeviceNode = hDevCookie;

	PVR_UNREFERENCED_PARAMETER(hDevCookie);

	if (!psMemInfo)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	eError = ResManDissociateRes(psMemInfo->sMemBlk.hResItem, psDeviceNode->hResManContext);

	PVR_ASSERT(eError == PVRSRV_OK);

	return eError;
}


/*!
******************************************************************************

 @Function	PVRSRVGetFreeDeviceMemKM

 @Description

 Determines how much memory remains available in the system with the specified
 capabilities.

 @Input	   ui32Flags :

 @Output   pui32Total :

 @Output   pui32Free :

 @Output   pui32LargestBlock :

 @Return   PVRSRV_ERROR  :

******************************************************************************/
IMG_EXPORT
PVRSRV_ERROR IMG_CALLCONV PVRSRVGetFreeDeviceMemKM(IMG_UINT32 ui32Flags,
												   IMG_SIZE_T *pui32Total,
												   IMG_SIZE_T *pui32Free,
												   IMG_SIZE_T *pui32LargestBlock)
{
	/* TO BE IMPLEMENTED */

	PVR_UNREFERENCED_PARAMETER(ui32Flags);
	PVR_UNREFERENCED_PARAMETER(pui32Total);
	PVR_UNREFERENCED_PARAMETER(pui32Free);
	PVR_UNREFERENCED_PARAMETER(pui32LargestBlock);

	return PVRSRV_OK;
}




/*!
******************************************************************************
	@Function   PVRSRVUnwrapExtMemoryKM

	@Description  On last unwrap of a given meminfo, unmaps physical pages from a
				wrapped allocation, and frees the associated device address space.
				Note: this can only unmap memory mapped by PVRSRVWrapExtMemory

	@Input	    psMemInfo - mem info describing the wrapped allocation
	@Return     None
******************************************************************************/
IMG_EXPORT
PVRSRV_ERROR IMG_CALLCONV PVRSRVUnwrapExtMemoryKM (PVRSRV_KERNEL_MEM_INFO	*psMemInfo)
{
	if (!psMemInfo)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	return ResManFreeResByPtr(psMemInfo->sMemBlk.hResItem);
}


static PVRSRV_ERROR UnwrapExtMemoryCallBack(IMG_PVOID	pvParam,
											IMG_UINT32	ui32Param)
{
	PVRSRV_KERNEL_MEM_INFO *psMemInfo = pvParam;

	return FreeMemCallBackCommon(psMemInfo, ui32Param, IMG_TRUE);
}


/*!
******************************************************************************
	@Function   PVRSRVWrapExtMemoryKM

	@Description  Allocates a Device Virtual Address in the shared mapping heap
				and maps physical pages into that allocation. Note, if the pages are
				already mapped into the heap, the existing allocation is returned.

	@Input	    hDevCookie - Device cookie
	@Input	    psPerProc - Per-process data
	@Input	    hDevMemContext - device memory context
	@Input	    uByteSize - Size of allocation
	@Input	    uPageOffset - Offset into the first page of the memory to be wrapped
	@Input	    bPhysContig - whether the underlying memory is physically contiguous
	@Input	    psExtSysPAddr - The list of Device Physical page addresses
	@Input	    pvLinAddr - ptr to buffer to wrap
	@Output     ppsMemInfo - mem info describing the wrapped allocation
	@Return     None
******************************************************************************/

IMG_EXPORT
PVRSRV_ERROR IMG_CALLCONV PVRSRVWrapExtMemoryKM(IMG_HANDLE				hDevCookie,
												PVRSRV_PER_PROCESS_DATA	*psPerProc,
												IMG_HANDLE				hDevMemContext,
												IMG_SIZE_T 				ui32ByteSize,
												IMG_SIZE_T				ui32PageOffset,
												IMG_BOOL				bPhysContig,
												IMG_SYS_PHYADDR	 		*psExtSysPAddr,
												IMG_VOID 				*pvLinAddr,
												IMG_UINT32				ui32Flags,
												PVRSRV_KERNEL_MEM_INFO	**ppsMemInfo)
{
	PVRSRV_KERNEL_MEM_INFO *psMemInfo = IMG_NULL;
	DEVICE_MEMORY_INFO  *psDevMemoryInfo;
	IMG_SIZE_T			ui32HostPageSize = HOST_PAGESIZE();
	IMG_HANDLE			hDevMemHeap = IMG_NULL;
	PVRSRV_DEVICE_NODE* psDeviceNode;
	BM_HANDLE 			hBuffer;
	PVRSRV_MEMBLK		*psMemBlock;
	IMG_BOOL			bBMError;
	BM_HEAP				*psBMHeap;
	PVRSRV_ERROR		eError;
	IMG_VOID 			*pvPageAlignedCPUVAddr;
	IMG_SYS_PHYADDR	 	*psIntSysPAddr = IMG_NULL;
	IMG_HANDLE			hOSWrapMem = IMG_NULL;
	DEVICE_MEMORY_HEAP_INFO *psDeviceMemoryHeap;
	IMG_SIZE_T		ui32PageCount = 0;
	IMG_UINT32		i;

	psDeviceNode = (PVRSRV_DEVICE_NODE*)hDevCookie;
	PVR_ASSERT(psDeviceNode != IMG_NULL);

	if (psDeviceNode == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "PVRSRVWrapExtMemoryKM: invalid parameter"));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	if(pvLinAddr)
	{

		ui32PageOffset = (IMG_UINTPTR_T)pvLinAddr & (ui32HostPageSize - 1);


		ui32PageCount = HOST_PAGEALIGN(ui32ByteSize + ui32PageOffset) / ui32HostPageSize;
		pvPageAlignedCPUVAddr = (IMG_VOID *)((IMG_UINTPTR_T)pvLinAddr - ui32PageOffset);


		if(OSAllocMem(PVRSRV_OS_PAGEABLE_HEAP,
						ui32PageCount * sizeof(IMG_SYS_PHYADDR),
						(IMG_VOID **)&psIntSysPAddr, IMG_NULL,
						"Array of Page Addresses") != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,"PVRSRVWrapExtMemoryKM: Failed to alloc memory for block"));
			return PVRSRV_ERROR_OUT_OF_MEMORY;
		}

		eError = OSAcquirePhysPageAddr(pvPageAlignedCPUVAddr,
										ui32PageCount * ui32HostPageSize,
										psIntSysPAddr,
										&hOSWrapMem,
										(ui32Flags != 0) ? IMG_TRUE : IMG_FALSE);
		if(eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,"PVRSRVWrapExtMemoryKM: Failed to alloc memory for block"));
			eError = PVRSRV_ERROR_OUT_OF_MEMORY;
			goto ErrorExitPhase1;
		}

		/* replace the supplied page address list */
		psExtSysPAddr = psIntSysPAddr;

		/* assume memory is not physically contiguous;
  		   we shouldn't trust what the user says here
  		*/
		bPhysContig = IMG_FALSE;
	}
	else
	{

	}


	psDevMemoryInfo = &((BM_CONTEXT*)hDevMemContext)->psDeviceNode->sDevMemoryInfo;
	psDeviceMemoryHeap = psDevMemoryInfo->psDeviceMemoryHeap;
	for(i=0; i<PVRSRV_MAX_CLIENT_HEAPS; i++)
	{
		if(HEAP_IDX(psDeviceMemoryHeap[i].ui32HeapID) == psDevMemoryInfo->ui32MappingHeapID)
		{
			if(psDeviceMemoryHeap[i].DevMemHeapType == DEVICE_MEMORY_HEAP_PERCONTEXT)
			{

				hDevMemHeap = BM_CreateHeap(hDevMemContext, &psDeviceMemoryHeap[i]);
			}
			else
			{
				hDevMemHeap = psDevMemoryInfo->psDeviceMemoryHeap[i].hDevMemHeap;
			}
			break;
		}
	}

	if(hDevMemHeap == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVWrapExtMemoryKM: unable to find mapping heap"));
		eError = PVRSRV_ERROR_GENERIC;
		goto ErrorExitPhase2;
	}

	if(OSAllocMem(PVRSRV_OS_PAGEABLE_HEAP,
					sizeof(PVRSRV_KERNEL_MEM_INFO),
					(IMG_VOID **)&psMemInfo, IMG_NULL,
					"Kernel Memory Info") != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVWrapExtMemoryKM: Failed to alloc memory for block"));
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto ErrorExitPhase2;
	}

	OSMemSet(psMemInfo, 0, sizeof(*psMemInfo));
	psMemInfo->ui32Flags = ui32Flags;

	psMemBlock = &(psMemInfo->sMemBlk);

	bBMError = BM_Wrap(hDevMemHeap,
					   ui32ByteSize,
					   ui32PageOffset,
					   bPhysContig,
					   psExtSysPAddr,
					   IMG_NULL,
					   &psMemInfo->ui32Flags,
					   &hBuffer);
	if (!bBMError)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVWrapExtMemoryKM: BM_Wrap Failed"));
		eError = PVRSRV_ERROR_BAD_MAPPING;
		goto ErrorExitPhase3;
	}

	/* Fill in "Implementation dependant" section of mem info */
	psMemBlock->sDevVirtAddr = BM_HandleToDevVaddr(hBuffer);
	psMemBlock->hOSMemHandle = BM_HandleToOSMemHandle(hBuffer);
	psMemBlock->hOSWrapMem = hOSWrapMem;
	psMemBlock->psIntSysPAddr = psIntSysPAddr;

	/* Convert from BM_HANDLE to external IMG_HANDLE */
	psMemBlock->hBuffer = (IMG_HANDLE)hBuffer;

	/* Fill in the public fields of the MEM_INFO structure */
	psMemInfo->pvLinAddrKM = BM_HandleToCpuVaddr(hBuffer);
	psMemInfo->sDevVAddr = psMemBlock->sDevVirtAddr;
	psMemInfo->ui32AllocSize = ui32ByteSize;

	/* Clear the Backup buffer pointer as we do not have one at this point.
	   We only allocate this as we are going up/down
	 */
	psMemInfo->pvSysBackupBuffer = IMG_NULL;

	/*
		allocate a syncinfo but don't register with resman
		because the holding devicemem will handle the syncinfo
	*/
	psBMHeap = (BM_HEAP*)hDevMemHeap;
	hDevMemContext = (IMG_HANDLE)psBMHeap->pBMContext;
	eError = PVRSRVAllocSyncInfoKM(hDevCookie,
									hDevMemContext,
									&psMemInfo->psKernelSyncInfo);
	if(eError != PVRSRV_OK)
	{
		goto ErrorExitPhase4;
	}

	psMemInfo->psKernelSyncInfo->ui32RefCount++;

	psMemInfo->memType = PVRSRV_MEMTYPE_WRAPPED;

	/* Register Resource */
	psMemInfo->sMemBlk.hResItem = ResManRegisterRes(psPerProc->hResManContext,
													RESMAN_TYPE_DEVICEMEM_WRAP,
													psMemInfo,
													0,
													UnwrapExtMemoryCallBack);

	/* return the meminfo */
	*ppsMemInfo = psMemInfo;

	return PVRSRV_OK;

	/* error handling: */

ErrorExitPhase4:
	if(psMemInfo)
	{
		FreeDeviceMem(psMemInfo);
		/*
			FreeDeviceMem will free the meminfo so set
			it to NULL to avoid double free below
		*/
		psMemInfo = IMG_NULL;
	}

ErrorExitPhase3:
	if(psMemInfo)
	{
		OSFreeMem(PVRSRV_OS_PAGEABLE_HEAP, sizeof(PVRSRV_KERNEL_MEM_INFO), psMemInfo, IMG_NULL);
		/*not nulling pointer, out of scope*/
	}

ErrorExitPhase2:
	if(psIntSysPAddr)
	{
		OSReleasePhysPageAddr(hOSWrapMem);
	}

ErrorExitPhase1:
	if(psIntSysPAddr)
	{
		OSFreeMem(PVRSRV_OS_PAGEABLE_HEAP, ui32PageCount * sizeof(IMG_SYS_PHYADDR), psIntSysPAddr, IMG_NULL);

	}

	return eError;
}


IMG_EXPORT
PVRSRV_ERROR IMG_CALLCONV PVRSRVUnmapDeviceMemoryKM (PVRSRV_KERNEL_MEM_INFO *psMemInfo)
{
	if (!psMemInfo)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	return ResManFreeResByPtr(psMemInfo->sMemBlk.hResItem);
}


static PVRSRV_ERROR UnmapDeviceMemoryCallBack(IMG_PVOID pvParam,
											  IMG_UINT32 ui32Param)
{
	PVRSRV_ERROR				eError;
	RESMAN_MAP_DEVICE_MEM_DATA	*psMapData = pvParam;

	PVR_UNREFERENCED_PARAMETER(ui32Param);

	if(psMapData->psMemInfo->sMemBlk.psIntSysPAddr)
	{
		OSFreeMem(PVRSRV_OS_PAGEABLE_HEAP, sizeof(IMG_SYS_PHYADDR), psMapData->psMemInfo->sMemBlk.psIntSysPAddr, IMG_NULL);
		psMapData->psMemInfo->sMemBlk.psIntSysPAddr = IMG_NULL;
	}

	psMapData->psMemInfo->psKernelSyncInfo->ui32RefCount--;
	if (psMapData->psMemInfo->psKernelSyncInfo->ui32RefCount == 0)
	{
		eError = PVRSRVFreeSyncInfoKM(psMapData->psMemInfo->psKernelSyncInfo);
		if(eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,"UnmapDeviceMemoryCallBack: Failed to free sync info"));
			return eError;
		}
	}

	eError = FreeDeviceMem(psMapData->psMemInfo);
	if(eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"UnmapDeviceMemoryCallBack: Failed to free DST meminfo"));
		return eError;
	}


	eError = FreeMemCallBackCommon(psMapData->psSrcMemInfo, 0, IMG_FALSE);

	OSFreeMem(PVRSRV_OS_PAGEABLE_HEAP, sizeof(RESMAN_MAP_DEVICE_MEM_DATA), psMapData, IMG_NULL);


	return eError;
}


/*!
******************************************************************************

 @Function	PVRSRVMapDeviceMemoryKM

 @Description
 		Maps an existing allocation to a specific device address space and heap
 		Note: it's valid to map from one physical device to another

 @Input	   psPerProc : Per-process data
 @Input    psSrcMemInfo
 @Input    hDstDevMemHeap
 @Input    ppsDstMemInfo

 @Return   PVRSRV_ERROR :

******************************************************************************/
IMG_EXPORT
PVRSRV_ERROR IMG_CALLCONV PVRSRVMapDeviceMemoryKM(PVRSRV_PER_PROCESS_DATA	*psPerProc,
												  PVRSRV_KERNEL_MEM_INFO	*psSrcMemInfo,
												  IMG_HANDLE				hDstDevMemHeap,
												  PVRSRV_KERNEL_MEM_INFO	**ppsDstMemInfo)
{
	PVRSRV_ERROR				eError;
	IMG_UINT32					i;
	IMG_SIZE_T					ui32PageCount, ui32PageOffset;
	IMG_SIZE_T					ui32HostPageSize = HOST_PAGESIZE();
	IMG_SYS_PHYADDR				*psSysPAddr = IMG_NULL;
	IMG_DEV_PHYADDR				sDevPAddr;
	BM_BUF						*psBuf;
	IMG_DEV_VIRTADDR			sDevVAddr;
	PVRSRV_KERNEL_MEM_INFO		*psMemInfo = IMG_NULL;
	BM_HANDLE 					hBuffer;
	PVRSRV_MEMBLK				*psMemBlock;
	IMG_BOOL					bBMError;
	PVRSRV_DEVICE_NODE			*psDeviceNode;
	IMG_VOID 					*pvPageAlignedCPUVAddr;
	RESMAN_MAP_DEVICE_MEM_DATA	*psMapData = IMG_NULL;

	/* check params */
	if(!psSrcMemInfo || !hDstDevMemHeap || !ppsDstMemInfo)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVMapDeviceMemoryKM: invalid parameters"));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	/* initialise the Dst Meminfo to NULL*/
	*ppsDstMemInfo = IMG_NULL;

	ui32PageOffset = psSrcMemInfo->sDevVAddr.uiAddr & (ui32HostPageSize - 1);
	ui32PageCount = HOST_PAGEALIGN(psSrcMemInfo->ui32AllocSize + ui32PageOffset) / ui32HostPageSize;
	pvPageAlignedCPUVAddr = (IMG_VOID *)(psSrcMemInfo->sDevVAddr.uiAddr - ui32PageOffset);

	/*
		allocate array of SysPAddr to hold SRC allocation page addresses
	*/
	if(OSAllocMem(PVRSRV_OS_PAGEABLE_HEAP,
					ui32PageCount*sizeof(IMG_SYS_PHYADDR),
					(IMG_VOID **)&psSysPAddr, IMG_NULL,
					"Array of Page Addresses") != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVMapDeviceMemoryKM: Failed to alloc memory for block"));
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	psBuf = psSrcMemInfo->sMemBlk.hBuffer;

	/* get the device node */
	psDeviceNode = psBuf->pMapping->pBMHeap->pBMContext->psDeviceNode;


	sDevVAddr.uiAddr = psSrcMemInfo->sDevVAddr.uiAddr - IMG_CAST_TO_DEVVADDR_UINT(ui32PageOffset);
	for(i=0; i<ui32PageCount; i++)
	{
		BM_GetPhysPageAddr(psSrcMemInfo, sDevVAddr, &sDevPAddr);

		/* save the address */
		psSysPAddr[i] = SysDevPAddrToSysPAddr (psDeviceNode->sDevId.eDeviceType, sDevPAddr);

		/* advance the DevVaddr one page */
		sDevVAddr.uiAddr += IMG_CAST_TO_DEVVADDR_UINT(ui32HostPageSize);
	}

	/* allocate the resman map data */
	if(OSAllocMem(PVRSRV_OS_PAGEABLE_HEAP,
					sizeof(RESMAN_MAP_DEVICE_MEM_DATA),
					(IMG_VOID **)&psMapData, IMG_NULL,
					"Resource Manager Map Data") != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVMapDeviceMemoryKM: Failed to alloc resman map data"));
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto ErrorExit;
	}


	if(OSAllocMem(PVRSRV_PAGEABLE_SELECT,
					sizeof(PVRSRV_KERNEL_MEM_INFO),
					(IMG_VOID **)&psMemInfo, IMG_NULL,
					"Kernel Memory Info") != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVMapDeviceMemoryKM: Failed to alloc memory for block"));
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto ErrorExit;
	}

	OSMemSet(psMemInfo, 0, sizeof(*psMemInfo));
	psMemInfo->ui32Flags = psSrcMemInfo->ui32Flags;

	psMemBlock = &(psMemInfo->sMemBlk);

	bBMError = BM_Wrap(hDstDevMemHeap,
					   psSrcMemInfo->ui32AllocSize,
					   ui32PageOffset,
					   IMG_FALSE,
					   psSysPAddr,
					   pvPageAlignedCPUVAddr,
					   &psMemInfo->ui32Flags,
					   &hBuffer);

	if (!bBMError)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVMapDeviceMemoryKM: BM_Wrap Failed"));
		eError = PVRSRV_ERROR_BAD_MAPPING;
		goto ErrorExit;
	}

	/* Fill in "Implementation dependant" section of mem info */
	psMemBlock->sDevVirtAddr = BM_HandleToDevVaddr(hBuffer);
	psMemBlock->hOSMemHandle = BM_HandleToOSMemHandle(hBuffer);

	/* Convert from BM_HANDLE to external IMG_HANDLE */
	psMemBlock->hBuffer = (IMG_HANDLE)hBuffer;

	/* Store page list */
	psMemBlock->psIntSysPAddr = psSysPAddr;

	/* patch up the CPU VAddr into the meminfo */
	psMemInfo->pvLinAddrKM = psSrcMemInfo->pvLinAddrKM;

	/* Fill in the public fields of the MEM_INFO structure */
	psMemInfo->sDevVAddr = psMemBlock->sDevVirtAddr;
	psMemInfo->ui32AllocSize = psSrcMemInfo->ui32AllocSize;
	psMemInfo->psKernelSyncInfo = psSrcMemInfo->psKernelSyncInfo;


	psMemInfo->psKernelSyncInfo->ui32RefCount++;



	psMemInfo->pvSysBackupBuffer = IMG_NULL;


	psMemInfo->ui32RefCount++;


	psSrcMemInfo->ui32RefCount++;


	BM_Export(psSrcMemInfo->sMemBlk.hBuffer);

	psMemInfo->memType = PVRSRV_MEMTYPE_MAPPED;

	/* setup the resman map data */
	psMapData->psMemInfo = psMemInfo;
	psMapData->psSrcMemInfo = psSrcMemInfo;

	/* Register Resource */
	psMemInfo->sMemBlk.hResItem = ResManRegisterRes(psPerProc->hResManContext,
													RESMAN_TYPE_DEVICEMEM_MAPPING,
													psMapData,
													0,
													UnmapDeviceMemoryCallBack);

	*ppsDstMemInfo = psMemInfo;

	return PVRSRV_OK;

	/* error handling: */

ErrorExit:

	if(psSysPAddr)
	{
		/* Free the page address list */
		OSFreeMem(PVRSRV_OS_PAGEABLE_HEAP, sizeof(IMG_SYS_PHYADDR), psSysPAddr, IMG_NULL);

	}

	if(psMemInfo)
	{

		OSFreeMem(PVRSRV_PAGEABLE_SELECT, sizeof(PVRSRV_KERNEL_MEM_INFO), psMemInfo, IMG_NULL);

	}

	if(psMapData)
	{

		OSFreeMem(PVRSRV_PAGEABLE_SELECT, sizeof(RESMAN_MAP_DEVICE_MEM_DATA), psMapData, IMG_NULL);

	}

	return eError;
}


IMG_EXPORT
PVRSRV_ERROR IMG_CALLCONV PVRSRVUnmapDeviceClassMemoryKM(PVRSRV_KERNEL_MEM_INFO *psMemInfo)
{
	if (!psMemInfo)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	return ResManFreeResByPtr(psMemInfo->sMemBlk.hResItem);
}


static PVRSRV_ERROR UnmapDeviceClassMemoryCallBack(IMG_PVOID	pvParam,
												   IMG_UINT32	ui32Param)
{
	PVRSRV_KERNEL_MEM_INFO *psMemInfo = pvParam;

	return FreeMemCallBackCommon(psMemInfo, ui32Param, IMG_TRUE);
}


/*!
******************************************************************************
	@Function   PVRSRVMapDeviceClassMemoryKM

	@Description  maps physical pages for DeviceClass buffers into a devices
				address space at a specified and pre-allocated Device
				Virtual Address

	@Input	    psPerProc - Per-process data
	@Input	    hDevMemContext - Device memory context
	@Input	    hDeviceClassBuffer - Device Class Buffer (Surface) handle
	@Input	    hDevMemContext - device memory context to which mapping
										is made
	@Output     ppsMemInfo - mem info describing the mapped memory
	@Output     phOSMapInfo - OS specific mapping information
	@Return     None
******************************************************************************/
IMG_EXPORT
PVRSRV_ERROR IMG_CALLCONV PVRSRVMapDeviceClassMemoryKM(PVRSRV_PER_PROCESS_DATA	*psPerProc,
													   IMG_HANDLE				hDevMemContext,
													   IMG_HANDLE				hDeviceClassBuffer,
													   PVRSRV_KERNEL_MEM_INFO	**ppsMemInfo,
													   IMG_HANDLE				*phOSMapInfo)
{
	PVRSRV_ERROR eError;
	PVRSRV_KERNEL_MEM_INFO *psMemInfo;
	PVRSRV_DEVICECLASS_BUFFER *psDeviceClassBuffer;
	IMG_SYS_PHYADDR *psSysPAddr;
	IMG_VOID *pvCPUVAddr, *pvPageAlignedCPUVAddr;
	IMG_BOOL bPhysContig;
	BM_CONTEXT *psBMContext;
	DEVICE_MEMORY_INFO *psDevMemoryInfo;
	DEVICE_MEMORY_HEAP_INFO *psDeviceMemoryHeap;
	IMG_HANDLE hDevMemHeap = IMG_NULL;
	IMG_SIZE_T ui32ByteSize;
	IMG_SIZE_T ui32Offset;
	IMG_SIZE_T ui32PageSize = HOST_PAGESIZE();
	BM_HANDLE		hBuffer;
	PVRSRV_MEMBLK	*psMemBlock;
	IMG_BOOL		bBMError;
	IMG_UINT32 i;
	IMG_BOOL bMapped = IMG_FALSE;
	
	if(!hDeviceClassBuffer || !ppsMemInfo || !phOSMapInfo || !hDevMemContext)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVMapDeviceClassMemoryKM: invalid parameters"));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	psDeviceClassBuffer = (PVRSRV_DEVICECLASS_BUFFER*)hDeviceClassBuffer;

	/*
		call into external driver to get info so we can map a meminfo
		Notes:
		It's expected that third party displays will only support
		physically contiguous display surfaces.  However, it's possible
		a given display may have an MMU and therefore support non-contig'
		display surfaces.

		If surfaces are contiguous, ext driver should return:
		 - a CPU virtual address, or IMG_NULL where the surface is not mapped to CPU
		 - (optional) an OS Mapping handle for KM->UM surface mapping
		 - the size in bytes
		 - a single system physical address

		If surfaces are non-contiguous, ext driver should return:
		 - a CPU virtual address
		 - (optional) an OS Mapping handle for KM->UM surface mapping
		 - the size in bytes (must be multiple of 4kB)
		 - a list of system physical addresses (at 4kB intervals)
	*/
	eError = psDeviceClassBuffer->pfnGetBufferAddr(psDeviceClassBuffer->hExtDevice,
												   psDeviceClassBuffer->hExtBuffer,
												   &psSysPAddr,
												   &ui32ByteSize,
												   &pvCPUVAddr,
												   phOSMapInfo,
												   &bPhysContig,
												   &bMapped);
	if(eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVMapDeviceClassMemoryKM: unable to get buffer address"));
		return PVRSRV_ERROR_GENERIC;
	}


	psBMContext = (BM_CONTEXT*)psDeviceClassBuffer->hDevMemContext;
	psDevMemoryInfo = &psBMContext->psDeviceNode->sDevMemoryInfo;
	psDeviceMemoryHeap = psDevMemoryInfo->psDeviceMemoryHeap;
	for(i=0; i<PVRSRV_MAX_CLIENT_HEAPS; i++)
	{
		if(HEAP_IDX(psDeviceMemoryHeap[i].ui32HeapID) == psDevMemoryInfo->ui32MappingHeapID)
		{
			if(psDeviceMemoryHeap[i].DevMemHeapType == DEVICE_MEMORY_HEAP_PERCONTEXT)
			{

				hDevMemHeap = BM_CreateHeap(hDevMemContext, &psDeviceMemoryHeap[i]);
			}
			else
			{
				hDevMemHeap = psDevMemoryInfo->psDeviceMemoryHeap[i].hDevMemHeap;
			}
			break;
		}
	}

	if(hDevMemHeap == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVMapDeviceClassMemoryKM: unable to find mapping heap"));
		return PVRSRV_ERROR_GENERIC;
	}


	ui32Offset = ((IMG_UINTPTR_T)pvCPUVAddr) & (ui32PageSize - 1);
	pvPageAlignedCPUVAddr = (IMG_VOID *)((IMG_UINTPTR_T)pvCPUVAddr - ui32Offset);

	if(OSAllocMem(PVRSRV_PAGEABLE_SELECT,
					sizeof(PVRSRV_KERNEL_MEM_INFO),
					(IMG_VOID **)&psMemInfo, IMG_NULL,
					"Kernel Memory Info") != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVMapDeviceClassMemoryKM: Failed to alloc memory for block"));
		return (PVRSRV_ERROR_OUT_OF_MEMORY);
	}

	OSMemSet(psMemInfo, 0, sizeof(*psMemInfo));

	psMemBlock = &(psMemInfo->sMemBlk);

	bBMError = BM_Wrap(hDevMemHeap,
					   ui32ByteSize,
					   ui32Offset,
					   bPhysContig,
					   psSysPAddr,
					   pvPageAlignedCPUVAddr,
					   &psMemInfo->ui32Flags,
					   &hBuffer);

	if (!bBMError)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVMapDeviceClassMemoryKM: BM_Wrap Failed"));
		OSFreeMem(PVRSRV_PAGEABLE_SELECT, sizeof(PVRSRV_KERNEL_MEM_INFO), psMemInfo, IMG_NULL);

		return PVRSRV_ERROR_BAD_MAPPING;
	}


	psMemBlock->sDevVirtAddr = BM_HandleToDevVaddr(hBuffer);
	psMemBlock->hOSMemHandle = BM_HandleToOSMemHandle(hBuffer);


	psMemBlock->hBuffer = (IMG_HANDLE)hBuffer;



	psMemInfo->pvLinAddrKM = BM_HandleToCpuVaddr(hBuffer);
	/* For Buffer Class of Texture Stream. Memory has been already mapped. */
	if (IMG_FALSE == bMapped) { 
		psMemInfo->sDevVAddr = psMemBlock->sDevVirtAddr;
	} else {
		psMemInfo->sDevVAddr.uiAddr = (IMG_UINT32)(psSysPAddr->uiAddr);
	}
	
	psMemInfo->ui32AllocSize = ui32ByteSize;
	psMemInfo->psKernelSyncInfo = psDeviceClassBuffer->psKernelSyncInfo;



	psMemInfo->pvSysBackupBuffer = IMG_NULL;


	psMemInfo->sMemBlk.hResItem = ResManRegisterRes(psPerProc->hResManContext,
													RESMAN_TYPE_DEVICECLASSMEM_MAPPING,
													psMemInfo,
													0,
													UnmapDeviceClassMemoryCallBack);

	psMemInfo->ui32RefCount++;

	psMemInfo->memType = PVRSRV_MEMTYPE_DEVICECLASS;

	/* return the meminfo */
	*ppsMemInfo = psMemInfo;

	return PVRSRV_OK;
}


/*
 * PVRSRVGetPageListKM()
 *
 * This is an EMGD addition to PVR services.  Given a PowerVR meminfo,
 * returns a list of pages for that allocation.  This can then be used
 * by EMGD code for things like mapping into the GTT if the surface
 * is going to be displayed.
 *
 * Note that the page list returned is the live page list and should
 * not be modified or freed by the caller.
 */
IMG_EXPORT
PVRSRV_ERROR IMG_CALLCONV PVRSRVGetPageListKM(PVRSRV_KERNEL_MEM_INFO *psMemInfo,
	struct page ***pvPageList,
	unsigned long *numpages,
	unsigned long *offset)
{
	LinuxMemArea *ma;
	unsigned long skippages, total_offset;

	/* Sanity check the parameters */
	if (!psMemInfo || !pvPageList || !numpages) {
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVMapDeviceMemoryKM: invalid parameters"));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	ma = (LinuxMemArea*)psMemInfo->sMemBlk.hOSMemHandle;

	/* # of pages isn't stored; needs to be calculated from number of bytes */
	*numpages = (ma->ui32ByteSize + 4095) / 4096;

	/*
	 * What type of memarea is this?  We can handle ALLOC_PAGES, or SUB_ALLOC
	 * areas whose earliest ancestor is an ALLOC_PAGES.
	 */
	switch (ma->eAreaType) {
	case LINUX_MEM_AREA_ALLOC_PAGES:
		*pvPageList = ma->uData.sPageList.pvPageList;
		*offset = 0;
		break;
	case LINUX_MEM_AREA_SUB_ALLOC:
		/*
		 * This allocation may be a subarea of a larger allocation.  We'll need to
		 * figure out the details of the parent allocation first so that we can
		 * calculate our subset of the page list and appropriate offset into the
		 * first page.
		 */
		total_offset = 0;
		while (ma->eAreaType == LINUX_MEM_AREA_SUB_ALLOC) {
			total_offset += ma->uData.sSubAlloc.ui32ByteOffset;
			ma = ma->uData.sSubAlloc.psParentLinuxMemArea;
		}

		/*
		 * We should now have the original ALLOC_PAGES memarea.  Make sure
		 * it's actually the type we expect.
		 */
		if (ma->eAreaType != LINUX_MEM_AREA_ALLOC_PAGES) {
			PVR_DPF((PVR_DBG_ERROR,
				"PVRSRVGetPageListKM: meminfo for suballocation did not "
				"originate from a a page-based allocation (type=%d)",
				ma->eAreaType));
			*numpages = 0;
			return PVRSRV_ERROR_GENERIC;
		}

		/*
		 * After taking all nested suballocations into account, figure out
		 * where in the page list the allocation we care about really starts.
		 */
		skippages = total_offset / 4096;
		*offset = total_offset % 4096;
		*pvPageList = &(ma->uData.sPageList.pvPageList)[skippages];

		break;

	default:
		PVR_DPF((PVR_DBG_ERROR,
			"PVRSRVGetPageListKM: meminfo not a page-based allocation or sub-allocation (type=%d)",
			ma->eAreaType));
		*numpages = 0;
		return PVRSRV_ERROR_GENERIC;
	}

	return PVRSRV_OK;
}


/******************************************************************************
 End of file (devicemem.c)
******************************************************************************/
