#!/usr/bin/env bash
# SPIKE (ADACPP_WASM_SHARED_OCCT): OCCT source patch for a default-visibility side module.
#
# OCCT's EM_JS imports (OSD_MemInfo heap size, Message_PrinterSystemLog console.*) are then
# default-visibility undefined functions, so a PIC link gives each a GOT.func entry. emscripten
# 4.0.9's loader (libdylink.js) resolves GOT entries in reportUndefinedSymbols() BEFORE it creates
# the module's own EM_JS functions from the __em_js__* exports, so libocct.so failed to load:
#   bad export type for 'OSD_MemInfo_getModuleHeapLength': undefined
# A hidden prototype ahead of each EM_JS makes the call DSO-local (no GOT entry); the __em_js__*
# string stays exported, so the loader still builds the JS function. (The static, hidden OCCT never
# hit this: hidden is the default there.)
set -euo pipefail
src="$1"
perl -0pi -e 's{(//! Print message to console\.debug\(\)\.)}{extern "C" __attribute__((visibility("hidden"))) void occJSConsoleDebug(const char*);\nextern "C" __attribute__((visibility("hidden"))) void occJSConsoleInfo(const char*);\nextern "C" __attribute__((visibility("hidden"))) void occJSConsoleWarn(const char*);\nextern "C" __attribute__((visibility("hidden"))) void occJSConsoleError(const char*);\n$1}' \
    "$src/src/FoundationClasses/TKernel/Message/Message_PrinterSystemLog.cxx"
perl -0pi -e 's{(//! Return WebAssembly heap size in bytes\.)}{extern "C" __attribute__((visibility("hidden"))) double OSD_MemInfo_getModuleHeapLength();\n$1}' \
    "$src/src/FoundationClasses/TKernel/OSD/OSD_MemInfo.cxx"
grep -q 'visibility("hidden"))) double OSD_MemInfo_getModuleHeapLength' "$src/src/FoundationClasses/TKernel/OSD/OSD_MemInfo.cxx"
grep -q 'visibility("hidden"))) void occJSConsoleError' "$src/src/FoundationClasses/TKernel/Message/Message_PrinterSystemLog.cxx"
