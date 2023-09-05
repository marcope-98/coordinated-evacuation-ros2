#include "rpl/map/AVLTree.hpp"

#include <algorithm>
#include <iostream>

#include <immintrin.h>

#include "rpl/internal/rplintrin.hpp"

rpl::AVLTree::AVLTree(const std::size_t &N) : d_minseg{N}, d_size{N}
{
  const std::size_t keys_size = this->align_up(this->d_size, 4);
  const std::size_t vals_size = this->align_up(this->d_size, 16);

  this->p_keys = new float[keys_size];
  this->p_vals = new std::uint8_t[vals_size]();

  __m128 two = _mm_set1_ps(2.f);
  for (std::size_t i = 0; i < keys_size; i += 4)
    _mm_store_ps(this->p_keys + i, two);
}

rpl::AVLTree::~AVLTree() { this->deallocate_all(); }

void rpl::AVLTree::deallocate_all()
{
  delete[] this->p_keys;
  delete[] this->p_vals;
}

void rpl::AVLTree::reset()
{
  this->d_minseg = this->d_size;

  const std::size_t keys_size = this->align_up(this->d_size, 4);
  const std::size_t vals_size = this->align_up(this->d_size, 16);

  __m128 two = _mm_set1_ps(2.f);
  for (std::size_t i = 0; i < keys_size; i += 4)
    _mm_store_ps(this->p_keys + i, two);

  __m128i zero = _mm_setzero_si128();
  for (std::size_t i = 0; i < vals_size; i += 16)
    _mm_store_si128((__m128i *)(this->p_vals + i), zero);
}

std::size_t rpl::AVLTree::minimum() const
{
  const __m128i increment  = _mm_set1_epi32(4);
  __m128i       indices    = _mm_set_epi32(3, 2, 1, 0);
  __m128i       minindices = indices;
  __m128        minvalues  = _mm_load_ps(this->p_keys);

  __m128  values;
  __m128i mask;
  for (std::size_t i = 4; i < this->size(); i += 4)
  {
    indices    = _mm_add_epi32(indices, increment);
    values     = _mm_load_ps(this->p_keys + i);
    mask       = _mm_castps_si128(_mm_cmplt_ps(values, minvalues));
    minindices = _mm_blendv_epi8(minindices, indices, mask);
    minvalues  = _mm_min_ps(values, minvalues);
  }

  float         values_array[4];
  std::uint32_t indices_array[4];
  _mm_store_ps(values_array, minvalues);
  _mm_store_si128((__m128i *)indices_array, minindices);
  std::size_t minindex = indices_array[0];
  float       minvalue = values_array[0];
  for (std::size_t i = 1; i < 4; ++i)
    if (ops::cmplt_f32(values_array[i], minvalue))
    {
      minindex = indices_array[i];
      minvalue = values_array[i];
    }

  return minindex > this->d_size ? this->d_size : minindex;
}

void rpl::AVLTree::resize(const std::size_t &newsize)
{
  delete[] this->p_keys;
  this->p_keys = new float[this->align_up(newsize, 4)];
  if (newsize > this->align_up(this->d_size, 16))
  {
    delete[] this->p_vals;
    this->p_vals = new std::uint8_t[this->align_up(newsize, 16)]();
  }

  this->d_size = newsize;
  this->reset();
}

void rpl::AVLTree::insert(const std::size_t &value, const float &key)
{
  const std::size_t div = value >> 3u;
  const std::size_t rem = value - (div << 3u);
  this->p_vals[div] |= (0x80u >> rem);
  this->p_keys[value] = key;
}

void rpl::AVLTree::remove(const std::size_t &value)
{
  const std::size_t div = value >> 3u;
  const std::size_t rem = value - (div << 3u);
  this->p_vals[div] &= ~(0x80u >> rem);
  this->p_keys[value] = 2.f;
}

bool rpl::AVLTree::is_set(const std::size_t &value) const
{
  const std::size_t div = value >> 3u;
  const std::size_t rem = value - (div << 3u);
  return bool(this->p_vals[div] & (0x80u >> rem));
}

void rpl::AVLTree::debug_info() const
{
  for (std::size_t i = 0; i < this->d_size; ++i)
    std::cerr << i << (this->is_set(i) ? " true  " : " false ") << this->p_keys[i] << "\n";
  std::cerr << "\n";
}
