// --post-js for the WASMFS modules: FS.writeFile that replaces the file.
//
// emscripten 4.0.9's WASMFS FS.writeFile (_wasmfs_write_file in system/lib/wasmfs/js_api.cpp) opens
// an existing file WITHOUT truncating it and writes at its current size: writing the same path twice
// leaves old + new concatenated, so a converter fed "/in.step" after an earlier input that was never
// unlinked reads a corrupt file. It also returns 0 instead of throwing when the file cannot be created,
// and stages the whole payload in the wasm heap through a byte-by-byte copy.
//
// This one is open("w") (O_TRUNC) + chunked write + close: replace semantics, an ErrnoError on
// failure like every other FS call, and at most one chunk staged in the heap at a time.
FS.writeFile = (path, data) => {
  if (typeof data == 'string') data = new TextEncoder().encode(data);
  var stream = FS.open(path, 'w');
  try {
    var chunk = 8 * 1024 * 1024;
    for (var pos = 0; pos < data.length; pos += chunk) {
      FS.write(stream, data, pos, Math.min(chunk, data.length - pos), pos);
    }
  } finally {
    FS.close(stream);
  }
  return data.length;
};
