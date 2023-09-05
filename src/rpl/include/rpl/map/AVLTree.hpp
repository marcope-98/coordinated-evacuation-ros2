#ifndef RPL_MAP_AVLTREE_HPP_
#define RPL_MAP_AVLTREE_HPP_
#include <cstdint>

namespace rpl
{
  struct AVLTree
  {
  public:
    // Constructors
    AVLTree() = default;
    explicit AVLTree(const std::size_t &N);
    AVLTree(const AVLTree &) = delete;
    AVLTree(AVLTree &&)      = delete;
    ~AVLTree();

    // Manipulators
    AVLTree &operator=(const AVLTree &) = delete;
    AVLTree &operator=(AVLTree &&) = delete;
    float    operator[](const std::size_t &value) const { return this->p_keys[value]; };
    float &  operator[](const std::size_t &value) { return this->p_keys[value]; };

    // Getters
    bool        empty() const { return this->d_minseg == this->d_size; }
    bool        is_set(const std::size_t &value) const;
    std::size_t minseg() const { return this->d_minseg; }
    std::size_t size() const { return this->d_size; }
    // std::size_t capacity() const { return (this->d_size + 3u) & ~(3u); }

    // Methods
    void insert(const std::size_t &value, const float &key);
    void remove(const std::size_t &value);
    void reset();
    void update_minseg() { this->d_minseg = this->minimum(); }
    void resize(const std::size_t &newsize);
    void debug_info() const;

  private:
    std::size_t   d_minseg{0};
    std::size_t   d_size{0};
    float *       p_keys{nullptr};
    std::uint8_t *p_vals{nullptr};

  private:
    std::size_t minimum() const;
    std::size_t align_up(const std::size_t &value, const std::size_t &bytes) const { return (value + (bytes - 1)) & ~(bytes - 1); }
    void        deallocate_all();
  };
} // namespace rpl
#endif // RPL_MAP_AVLTREE_HPP_