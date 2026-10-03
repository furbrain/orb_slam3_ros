#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct TarHeader {                       // offset
  char name[100];                        //   0
  char mode[8];                          // 100
  char uid[8];                           // 108
  char gid[8];                           // 116
  char size[12];                         // 124
  char mtime[12];                        // 136
  char chksum[8];                        // 148
  char typeflag;                         // 156
  char linkname[100];                    // 157
  char magic[6];                         // 257
  char version[2];                       // 263
  char uname[32];                        // 265
  char gname[32];                        // 297
  char devmajor[8];                      // 329
  char devminor[8];                      // 337
  char prefix[155];                      // 345
  char padding[12];                      // 500
};
static_assert(sizeof(TarHeader) == 512, "unexpected tar header size");

void tar_write_file(std::ostream &out, std::string const &filename,
                    char const *data, std::size_t size, std::uint64_t mtime = 0) {
  TarHeader h;
  if (mtime == 0) {
    mtime = std::time(0);
  }
  std::memset(&h, 0, sizeof h);

  std::strncpy(h.name, filename.c_str(), sizeof h.name - 1);
  std::memcpy(h.mode, "0000644", 7);
  std::memcpy(h.uid,  "0000000", 7);
  std::memcpy(h.gid,  "0000000", 7);
  std::snprintf(h.size,  sizeof h.size,  "%011llo", static_cast<unsigned long long>(size));
  std::snprintf(h.mtime, sizeof h.mtime, "%011llo", static_cast<unsigned long long>(mtime));
  std::memset(h.chksum, ' ', sizeof h.chksum);   // must be spaces while summing
  h.typeflag = '0';
  std::memcpy(h.magic, "ustar ", 6);
  std::memcpy(h.version, " ", 2);                // ' ' then '\0'
  std::strncpy(h.uname, "root", sizeof h.uname - 1);
  std::strncpy(h.gname, "root", sizeof h.gname - 1);

  unsigned int sum = 0;
  auto const *bytes = reinterpret_cast<unsigned char const *>(&h);
  for (std::size_t i = 0; i < sizeof h; ++i) sum += bytes[i];
  std::snprintf(h.chksum, sizeof h.chksum, "%06o", sum);  // 6 digits, '\0', then the ' ' left over

  out.write(reinterpret_cast<char const *>(&h), sizeof h);
  out.write(data, static_cast<std::streamsize>(size));
  std::size_t const pad = (512u - size % 512u) & 511u;
  if (pad) out.write(std::string(pad, '\0').data(), static_cast<std::streamsize>(pad));
}

void tar_write_tail(std::ostream &out) {
  out.write(std::string(1024, '\0').data(), 1024);
}

}  // namespace
