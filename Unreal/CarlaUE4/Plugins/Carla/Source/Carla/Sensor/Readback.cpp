#include "Readback.h"
#ifdef _WIN32
#include <Windows.h>
#pragma comment(lib, "Synchronization.lib")
#endif

FTextureReadback::FTextureReadback() :
	StagingTexture(), bDone()
{
}

FTextureReadback::FTextureReadback(FTextureReadback&& other) noexcept
{
	(void)memcpy(this, &other, sizeof(FTextureReadback));
	(void)memset(&other, 0, sizeof(FTextureReadback));
}

FTextureReadback& FTextureReadback::operator=(FTextureReadback&& other) noexcept
{
	this->~FTextureReadback();
	new (this) FTextureReadback(std::move(other));
	return *this;
}

void FTextureReadback::EnqueueCopy(FRHICommandList& RHICmdList, FTextureRHIRef SourceTexture, FResolveRect Rect)
{
	if (SourceTexture != nullptr)
	{
		if (StagingTexture == nullptr)
		{
			auto Size = SourceTexture->GetSizeXYZ();
			FRHIResourceCreateInfo CreateInfo = {};
			StagingTexture = RHICreateTexture2D(
				Size.X, Size.Y,
				SourceTexture->GetFormat(),
				1, 1,
				TexCreate_CPUReadback | TexCreate_HideInVisualizeTexture,
				CreateInfo);
		}
		RHICmdList.Transition(FRHITransitionInfo(StagingTexture, ERHIAccess::Unknown, ERHIAccess::CopyDest));
		FResolveParams ResolveParams(Rect);
		ResolveParams.SourceAccessFinal = ERHIAccess::Unknown;
		ResolveParams.DestAccessFinal = ERHIAccess::Unknown;
		RHICmdList.CopyToResolveTarget(SourceTexture, StagingTexture, ResolveParams);
		RHICmdList.Transition(FRHITransitionInfo(StagingTexture, ERHIAccess::CopyDest, ERHIAccess::CPURead));
		bDone.store(true, std::memory_order_relaxed);
#ifdef _WIN32
		WakeByAddressAll(&bDone);
#endif
	}
}

bool FTextureReadback::IsReady()
{
	return bDone.load(std::memory_order_acquire);
}

void FTextureReadback::Await()
{
	while (true)
	{
		auto prior = bDone.load(std::memory_order_acquire);
		if (prior)
			break;
		bool expected = prior;
#ifdef _WIN32
		WaitOnAddress(&bDone, &expected, 1, INFINITE);
#endif
	}
}

void* FTextureReadback::Lock(FRHICommandListImmediate& RHICmdList, FIntPoint& Extent)
{
	check(bDone.load(std::memory_order_acquire));
	void* Result = nullptr;
	FGPUFenceRHIRef Fence = {};
	RHICmdList.MapStagingSurface(StagingTexture, Fence.GetReference(), Result, Extent.X, Extent.Y);
	return Result;
}

void FTextureReadback::Unlock(FRHICommandListImmediate& RHICmdList)
{
	check(bDone.load(std::memory_order_acquire));
	RHICmdList.UnmapStagingSurface(StagingTexture);
}