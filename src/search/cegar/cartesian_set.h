#ifndef CEGAR_CARTESIAN_SET_H
#define CEGAR_CARTESIAN_SET_H

#include "../algorithms/dynamic_bitset.h"

#include <ostream>
#include <vector>

namespace cegar {
using Bitset = dynamic_bitset::DynamicBitset<unsigned short>;

/*
  For each variable store a subset of its domain.

  The underlying data structure is a vector of bitsets.
*/
class CartesianSet {
    std::vector<Bitset> domain_subsets;

public:
    explicit CartesianSet(const std::vector<int> &domain_sizes);

    void add(int var, int value);
    void set_single_value(int var, int value);
    void remove(int var, int value);
    void add_all(int var);
    void remove_all(int var);
    void intersect(int var, Bitset &vals);

    bool test(int var, int value) const {
        return domain_subsets[var][value];
    }

    int count(int var) const;
    bool intersects(const CartesianSet &other) const;
    bool intersects(const CartesianSet &other, int var) const;
    bool is_superset_of(const CartesianSet &other) const;

    bool is_disjunct(int var, Bitset &vals) const;

    Bitset get_bitset_from_var(int var) const;

    Bitset& get_bitset_ref_from_var(int var);

    void set_bitset(int var, Bitset vals);

    size_t size() const;

    bool validate();

    friend std::ostream &operator<<(
        std::ostream &os, const CartesianSet &cartesian_set);
};
}

#endif
