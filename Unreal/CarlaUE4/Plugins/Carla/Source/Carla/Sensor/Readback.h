#pragma once
#include "CoreMinimal.h"
#include "RHI.h"
#include <atomic>



class FTextureReadback
{
public:

	FTextureReadback();
	FTextureReadback(FTextureReadback&& other) noexcept;
	FTextureReadback& operator=(FTextureReadback&& other) noexcept;
	~FTextureReadback() = default;

	void EnqueueCopy(FRHICommandList& RHICmdList, FTextureRHIRef SourceTexture, FResolveRect Rect = FResolveRect());
	bool IsReady();
	void Await();
	void* Lock(FRHICommandListImmediate& RHICmdList, FIntPoint& Extent);
	void Unlock(FRHICommandListImmediate& RHICmdList);

	FTextureRHIRef StagingTexture;
	std::atomic_bool bDone;
};