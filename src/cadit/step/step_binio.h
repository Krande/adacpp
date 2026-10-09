#pragma once
// Minimal little-endian binary stream for persisting reader state (the StreamIndex offset table and
// the resolvers' read-only metadata maps) to a file another process or wasm instance loads instead of
// re-scanning the source. Host byte order: the writer and reader are the same build (one browser's
// workers, or one server's processes), so no endian or versioning ceremony beyond a magic tag.

#include <array>
#include <cstdint>
#include <fstream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace adacpp::step::binio {

class Out {
public:
    explicit Out(const std::string &path) : f_(path, std::ios::binary) {}
    explicit operator bool() const {
        return (bool) f_;
    }
    template <class T> std::enable_if_t<std::is_trivially_copyable_v<T>> pod(const T &v) {
        f_.write(reinterpret_cast<const char *>(&v), sizeof(T));
    }
    void u64(uint64_t v) {
        pod(v);
    }
    void str(const std::string &s) {
        u64(s.size());
        f_.write(s.data(), (std::streamsize) s.size());
    }
    template <class T> void vec(const std::vector<T> &v) {
        u64(v.size());
        if constexpr (std::is_trivially_copyable_v<T>)
            f_.write(reinterpret_cast<const char *>(v.data()), (std::streamsize) (v.size() * sizeof(T)));
        else
            for (const T &x : v)
                item(x);
    }
    template <class K, class V> void map(const std::unordered_map<K, V> &m) {
        u64(m.size());
        for (const auto &[k, v] : m) {
            item(k);
            item(v);
        }
    }
    template <class K> void set(const std::unordered_set<K> &s) {
        u64(s.size());
        for (const K &k : s)
            item(k);
    }

private:
    template <class T> void item(const T &x) {
        if constexpr (std::is_same_v<T, std::string>)
            str(x);
        else if constexpr (std::is_trivially_copyable_v<T>)
            pod(x);
        else
            nested(x);
    }
    template <class A, class B> void nested(const std::pair<A, B> &p) {
        item(p.first);
        item(p.second);
    }
    template <class T> void nested(const std::vector<T> &v) {
        vec(v);
    }
    std::ofstream f_;
};

// Every length read is checked against the bytes left in the file before anything is allocated, so a
// truncated or corrupt index fails (operator bool false, containers left empty) instead of attempting
// a huge allocation.
class In {
public:
    explicit In(const std::string &path) : f_(path, std::ios::binary | std::ios::ate) {
        if (f_) {
            size_ = (uint64_t) f_.tellg();
            f_.seekg(0);
        }
    }
    explicit operator bool() const {
        return (bool) f_;
    }
    template <class T> std::enable_if_t<std::is_trivially_copyable_v<T>> pod(T &v) {
        f_.read(reinterpret_cast<char *>(&v), sizeof(T));
    }
    uint64_t u64() {
        uint64_t v = 0;
        pod(v);
        return v;
    }
    void str(std::string &s) {
        const uint64_t n = u64();
        if (!fits(n, 1)) {
            s.clear();
            return;
        }
        s.resize(n);
        f_.read(s.data(), (std::streamsize) s.size());
    }
    template <class T> void vec(std::vector<T> &v) {
        const uint64_t n = u64();
        if (!fits(n, std::is_trivially_copyable_v<T> ? sizeof(T) : 1)) {
            v.clear();
            return;
        }
        v.resize(n);
        if constexpr (std::is_trivially_copyable_v<T>)
            f_.read(reinterpret_cast<char *>(v.data()), (std::streamsize) (v.size() * sizeof(T)));
        else
            for (T &x : v)
                item(x);
    }
    template <class K, class V> void map(std::unordered_map<K, V> &m) {
        const uint64_t n = u64();
        m.clear();
        if (!fits(n, 1))
            return;
        m.reserve(n);
        for (uint64_t i = 0; i < n && f_; ++i) {
            K k{};
            item(k);
            item(m[k]);
        }
    }
    template <class K> void set(std::unordered_set<K> &s) {
        const uint64_t n = u64();
        s.clear();
        if (!fits(n, 1))
            return;
        s.reserve(n);
        for (uint64_t i = 0; i < n && f_; ++i) {
            K k{};
            item(k);
            s.insert(k);
        }
    }

private:
    // n items of at least `each` bytes still fit in the file; else mark the stream failed.
    bool fits(uint64_t n, uint64_t each) {
        if (!f_)
            return false;
        const std::streamoff at = f_.tellg();
        const uint64_t left = at < 0 || (uint64_t) at > size_ ? 0 : size_ - (uint64_t) at;
        if (n > left / each) {
            f_.setstate(std::ios::failbit);
            return false;
        }
        return true;
    }
    template <class T> void item(T &x) {
        if constexpr (std::is_same_v<T, std::string>)
            str(x);
        else if constexpr (std::is_trivially_copyable_v<T>)
            pod(x);
        else
            nested(x);
    }
    template <class A, class B> void nested(std::pair<A, B> &p) {
        item(p.first);
        item(p.second);
    }
    template <class T> void nested(std::vector<T> &v) {
        vec(v);
    }
    std::ifstream f_;
    uint64_t size_ = 0;
};

} // namespace adacpp::step::binio
