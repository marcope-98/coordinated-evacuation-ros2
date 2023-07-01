#ifndef RPL_MAP_AVLTREE_HPP_
#define RPL_MAP_AVLTREE_HPP_

#include <cstdint>
#include <utility>

namespace rpl
{
  struct AVLTree
  {
    // Member variables
    std::size_t   minseg      = 0;
    std::size_t   size        = 0;
    float *       keys        = nullptr;
    std::uint8_t *ActiveEdges = nullptr;

    // Constructors
    AVLTree() = default;
    explicit AVLTree(const std::size_t &N);
    AVLTree(const AVLTree &other) { *this = other; }
    AVLTree(AVLTree &&other) { *this = std::move(other); }
    ~AVLTree();

    // Manipulators
    AVLTree &operator=(const AVLTree &other);
    AVLTree &operator=(AVLTree &&other);

  public:
    // Getters
    bool empty() const { return this->minseg == this->size; }
    bool is_set(const std::size_t &value) const;

    // Methods
    void insert(const std::size_t &value, const float &key);
    void remove(const std::size_t &value);
    void update(const std::size_t &value, const float &key);
    void reset();
    void debug_info() const;
    void update_minseg();

  private:
    std::size_t minimum() const;
    std::size_t align_up(const std::size_t &bytes) const { return (this->size + (bytes - 1)) & ~(bytes - 1); }
    void        deallocate_all();
  };
} // namespace rpl

#endif // RPL_MAP_AVLTREE_HPP_
