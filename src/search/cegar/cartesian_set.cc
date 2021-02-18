#include "cartesian_set.h"

#include <sstream>

using namespace std;

namespace cegar {
CartesianSet::CartesianSet(const vector<int> &domain_sizes) {
    domain_subsets.reserve(domain_sizes.size());
    for (int domain_size : domain_sizes) {
        Bitset domain(domain_size);
        domain.set();
        domain_subsets.push_back(move(domain));
    }
}

void CartesianSet::add(int var, int value) {
    domain_subsets[var].set(value);
}

void CartesianSet::remove(int var, int value) {
    domain_subsets[var].reset(value);
}

void CartesianSet::set_single_value(int var, int value) {
    remove_all(var);
    add(var, value);
}

void CartesianSet::add_all(int var) {
    domain_subsets[var].set();
}

void CartesianSet::remove_all(int var) {
    domain_subsets[var].reset();
}

void CartesianSet::intersect(int var, Bitset &vals) {
    domain_subsets[var] &= vals;
}

int CartesianSet::count(int var) const {
    return domain_subsets[var].count();
}

bool CartesianSet::intersects(const CartesianSet &other) const {
    int num_vars = domain_subsets.size();
    for (int var = 0; var < num_vars; ++var) {
        if (!domain_subsets[var].intersects(other.domain_subsets[var])) {
            return false;
        }
    }
    return true;
}

bool CartesianSet::intersects(const CartesianSet &other, int var) const {
    return domain_subsets[var].intersects(other.domain_subsets[var]);
}

bool CartesianSet::is_superset_of(const CartesianSet &other) const {
    int num_vars = domain_subsets.size();
    for (int var = 0; var < num_vars; ++var) {
        if (!other.domain_subsets[var].is_subset_of(domain_subsets[var]))
            return false;
    }
    return true;
}

bool CartesianSet::is_disjunct(int var, Bitset &vals) const {
    return domain_subsets[var].is_disjunct(vals) ? true : false;
}

Bitset CartesianSet::get_bitset_from_var(int var) const {
    return domain_subsets[var];
}

Bitset& CartesianSet::get_bitset_ref_from_var(int var) {
    return domain_subsets[var];
}

void CartesianSet::set_bitset(int var, Bitset vals) {
    domain_subsets[var] = vals;
}

size_t CartesianSet::size() const {
    return domain_subsets.size();
}

bool CartesianSet::validate() {
    int num_vars = domain_subsets.size();
    for (int var = 0; var < num_vars; ++var) {
        if (domain_subsets[var].count() == 0)
            return false;
    }
    return true;
}

ostream &operator<<(ostream &os, const CartesianSet &cartesian_set) {
    int num_vars = cartesian_set.domain_subsets.size();
    string var_sep;
    os << "<";
    for (int var = 0; var < num_vars; ++var) {
        const Bitset &domain = cartesian_set.domain_subsets[var];
        vector<int> values;
        for (size_t value = 0; value < domain.size(); ++value) {
            if (domain[value])
                values.push_back(value);
        }
        if (values.empty()) {
            cout << endl << "error in variable: " << var << endl;
        }
        assert(!values.empty());
        if (values.size() < domain.size()) {
            os << var_sep << var << "={";
            string value_sep;
            for (int value : values) {
                os << value_sep << value;
                value_sep = ",";
            }
            os << "}";
            var_sep = ",";
        }
    }
    return os << ">";
}
}
