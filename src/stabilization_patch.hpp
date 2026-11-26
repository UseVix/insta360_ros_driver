// Disable the stabilization rotation that libMediaSDK's real-time stitcher applies to its output.
//
// The stitch shader rotates every output direction by F·(D·S)⁻¹·F before looking up the stitch
// maps, where S and D come from two virtual getters of the stitch render object:
//   DynamicStitchDoubleFisheyeStitchObject::GetStablizationQuat()   (stabilization quaternion)
//   SphericalRenderObject::GetDirectionalQuat()                     (directional quaternion)
// Replacing both in that class's vtable with functions returning identity makes the rotation
// identity, so the stitched image stays fixed to the camera (no stabilization, no smoothing).
// The stabilizer, gyro filter, flowstate / rolling-shutter maps and every other GPU input are
// unchanged; the stitch maps are indexed after the rotation, so they stay aligned.
//
// The vtable slots are located by their current contents (the exported addresses of the two
// originals), not by fixed offsets, and nothing is written unless each original is found exactly
// once. Install before RealTimeStitcher::StartStitch() so no render thread is using the vtable.
// The patch is process-wide: it affects every object of that class in this process.
#pragma once

#ifndef _GNU_SOURCE
#define _GNU_SOURCE  // dladdr1
#endif
#include <dlfcn.h>
#include <link.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <cstdint>
#include <string>

namespace insta360_ros_driver {


  // Returns true if the patch is active (installed now or earlier); on failure `error` says why.
  static bool install_patch(std::string& error) {
    static constexpr const char* GetStablizationQuat = "_ZN9arvrender38DynamicStitchDoubleFisheyeStitchObject19GetStablizationQuatEv";
    static constexpr const char* GetDirectionalQuat = "_ZN9arvrender21SphericalRenderObject18GetDirectionalQuatEv";
    static constexpr const char* DynamicStitchDoubleFisheyeStitchObject = "_ZTVN9arvrender38DynamicStitchDoubleFisheyeStitchObjectE";
    void* orig_stab = dlsym(RTLD_DEFAULT, GetStablizationQuat);
    void* orig_dir = dlsym(RTLD_DEFAULT, GetDirectionalQuat);
    auto* vtable = static_cast<void**>(dlsym(RTLD_DEFAULT, DynamicStitchDoubleFisheyeStitchObject));
    if (!orig_stab || !orig_dir || !vtable) {
      error = "libMediaSDK symbols not found (different SDK build?)";
      return false;
    }
    // Scan only this class's vtable: its size comes from the symbol table entry.
    Dl_info info{};
    void* sym_entry = nullptr;
    if (!dladdr1(vtable, &info, &sym_entry, RTLD_DL_SYMENT) || !sym_entry) {
      error = "could not determine the vtable size";
      return false;
    }
    const auto* sym = static_cast<const ElfW(Sym)*>(sym_entry);
    if (sym->st_size < 3 * sizeof(void*)) {
      error = "could not determine the vtable size";
      return false;
    }
    const int entries = static_cast<int>(sym->st_size / sizeof(void*));
    void** stab_slot = nullptr;
    void** dir_slot = nullptr;
    int stab_hits = 0, dir_hits = 0;
    // vtable layout: [offset-to-top][typeinfo][virtual functions...]
    for (int i = 2; i < entries; ++i) {
      if (vtable[i] == orig_stab) { stab_slot = &vtable[i]; ++stab_hits; }
      if (vtable[i] == orig_dir) { dir_slot = &vtable[i]; ++dir_hits; }
    }
    if (stab_hits != 1 || dir_hits != 1) {
      error = "unexpected vtable layout (GetStablizationQuat found " + std::to_string(stab_hits) +
              "x, GetDirectionalQuat found " + std::to_string(dir_hits) + "x); not patching";
      return false;
    }
    if (!write_slot(stab_slot, reinterpret_cast<void*>(&identity_quat), error) ||
        !write_slot(dir_slot, reinterpret_cast<void*>(&identity_quat), error)) {
      return false;
    }
    return true;
  }

  // Same ABI as the originals: `this` in rdi, glm::quat (4 floats x,y,z,w) returned in xmm0/xmm1.
  struct Quat { float x, y, z, w; };
  static Quat identity_quat(void*) {return {0.f, 0.f, 0.f, 1.f}; }
  static bool write_slot(void** slot, void* fn, std::string& error) {
    const long page = sysconf(_SC_PAGESIZE);
    auto* start = reinterpret_cast<void*>(reinterpret_cast<uintptr_t>(slot) & ~static_cast<uintptr_t>(page - 1));
    if (mprotect(start, page, PROT_READ | PROT_WRITE) != 0) {
      error = "mprotect(RW) failed on the vtable page";
      return false;
    }
    *slot = fn;
    mprotect(start, page, PROT_READ);
    return true;
  }



}  // namespace insta360_ros_driver
