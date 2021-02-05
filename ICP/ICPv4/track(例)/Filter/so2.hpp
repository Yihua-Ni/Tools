#ifndef LIE_SO2_H
#define LIE_SO2_H

#include <complex>
#include <Eigen/LU>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace Lie {

template <class Scalar, int M>
using Vector = Eigen::Matrix<Scalar, M, 1>;

template <class Scalar>
using Vector2 = Vector<Scalar, 2>;
using Vector2f = Vector2<float>;
using Vector2d = Vector2<double>;


template <class Scalar, int M, int N>
using Matrix = Eigen::Matrix<Scalar, M, N>;

template <class Scalar>
using Matrix2 = Matrix<Scalar, 2, 2>;
using Matrix2f = Matrix2<float>;
using Matrix2d = Matrix2<double>;


}  // namespace Lie

namespace Lie {
template <class Scalar_, int Options = 0>
class SO2;
using SO2d = SO2<double>;
using SO2f = SO2<float>;

template <class Scalar>
struct Constants {
   static Scalar epsilon() { return Scalar(1e-10); }

   static Scalar pi() { return Scalar(M_PI); }
};

template <>
struct Constants<float> {
   static float epsilon() { return static_cast<float>(1e-5); }

   static float pi() { return static_cast<float>(M_PI); }
};

}  // namespace Lie

namespace Eigen {
namespace internal {

template <class Scalar_, int Options>
struct traits<Lie::SO2<Scalar_, Options>> {
  using Scalar = Scalar_;
  using ComplexType = Lie::Vector2<Scalar>;
};

template <class Scalar_, int Options>
struct traits<Map<Lie::SO2<Scalar_>, Options>>
    : traits<Lie::SO2<Scalar_, Options>> {
  using Scalar = Scalar_;
  using ComplexType = Map<Lie::Vector2<Scalar>, Options>;
};

template <class Scalar_, int Options>
struct traits<Map<Lie::SO2<Scalar_> const, Options>>
    : traits<Lie::SO2<Scalar_, Options> const> {
  using Scalar = Scalar_;
  using ComplexType = Map<Lie::Vector2<Scalar> const, Options>;
};
}  // namespace internal
}  // namespace Eigen

namespace Lie {

//  | cos(theta) -sin(theta) |
//  | sin(theta)  cos(theta) |
//
template <class Derived>
class SO2Base {
 public:
  using Scalar = typename Eigen::internal::traits<Derived>::Scalar;
  using Complex = typename Eigen::internal::traits<Derived>::ComplexType;

  static int constexpr DoF = 1;
  static int constexpr num_parameters = 2;
  static int constexpr N = 2;
  using Transformation = Matrix<Scalar, N, N>;
  using Point = Vector2<Scalar>;
  using Tangent = Scalar;
  using Adjoint = Scalar;

  // Adjoint transformation
  // ``hat(Ad_A * x) = A * hat(x) A^{-1}``. See hat-operator below.
   Adjoint Adj() const { return 1; }

  template <class NewScalarType>
   SO2<NewScalarType> cast() const {
    return SO2<NewScalarType>(unit_complex().template cast<NewScalarType>());
  }

   Scalar* data() { return unit_complex_nonconst().data(); }

   Scalar const* data() const { return unit_complex().data(); }

   SO2<Scalar> inverse() const {
    return SO2<Scalar>(unit_complex().x(), -unit_complex().y());
  }

   Scalar log() const { return SO2<Scalar>::log(*this); }

   void normalize() {
    Scalar length = std::sqrt(unit_complex().x() * unit_complex().x() +
                              unit_complex().y() * unit_complex().y());
    assert(length >= Constants<Scalar>::epsilon());
    unit_complex_nonconst().x() /= length;
    unit_complex_nonconst().y() /= length;
  }

   Transformation matrix() const {
    Scalar const& real = unit_complex().x();
    Scalar const& imag = unit_complex().y();
    Transformation R;
        R <<
             real, -imag,
                imag,  real;
    return R;
  }

  template <class OtherDerived>
   SO2Base<Derived>& operator=(SO2Base<OtherDerived> const& other) {
    unit_complex_nonconst() = other.unit_complex();
    return *this;
  }

   SO2<Scalar> operator*(SO2<Scalar> const& other) const {
    SO2<Scalar> result(*this);
    result *= other;
    return result;
  }

   Point operator*(Point const& p) const {
    Scalar const& real = unit_complex().x();
    Scalar const& imag = unit_complex().y();
    return Point(real * p[0] - imag * p[1], imag * p[0] + real * p[1]);
  }

   SO2Base<Derived> operator*=(SO2<Scalar> const& other) {
    Scalar lhs_real = unit_complex().x();
    Scalar lhs_imag = unit_complex().y();
    Scalar const& rhs_real = other.unit_complex().x();
    Scalar const& rhs_imag = other.unit_complex().y();
    unit_complex_nonconst().x() = lhs_real * rhs_real - lhs_imag * rhs_imag;
    unit_complex_nonconst().y() = lhs_real * rhs_imag + lhs_imag * rhs_real;

    Scalar squared_norm = unit_complex_nonconst().squaredNorm();

    if (squared_norm != Scalar(1.0)) {
      unit_complex_nonconst() *= Scalar(2.0) / (Scalar(1.0) + squared_norm);
    }
    return *this;
  }

   void setComplex(Point const& complex) {
    unit_complex_nonconst() = complex;
    normalize();
  }

  Complex const& unit_complex() const {
    return static_cast<Derived const*>(this)->unit_complex();
  }

   static SO2<Scalar> exp(Tangent const& theta) {
    return SO2<Scalar>(std::cos(theta), std::sin(theta));
  }

   static Transformation generator() { return hat(1); }

   static Transformation hat(Tangent const& theta) {
    Transformation Omega;
    Omega <<
        Scalar(0),   -theta,
            theta, Scalar(0);
    return Omega;
  }

   static Tangent lieBracket(Tangent const&, Tangent const&) {
    return Scalar(0);
  }

   static Tangent log(SO2<Scalar> const& other) {
    using std::atan2;
    return atan2(other.unit_complex_.y(), other.unit_complex().x());
  }

   static Tangent vee(Transformation const& Omega) {
    using std::abs;
    assert(Omega.diagonal().template lpNorm<1>() < Constants<Scalar>::epsilon());
    assert(abs(Omega(1, 0) + Omega(0, 1)) < Constants<Scalar>::epsilon());
    return Omega(1, 0);
  }

 private:
  
  Complex& unit_complex_nonconst() {
    return static_cast<Derived*>(this)->unit_complex_nonconst();
  }
};

template <class Scalar_, int Options>
class SO2 : public SO2Base<SO2<Scalar_, Options>> {
  using Base = SO2Base<SO2<Scalar_, Options>>;

 public:
  using Scalar = Scalar_;
  using Transformation = typename Base::Transformation;
  using Point = typename Base::Point;
  using Tangent = typename Base::Tangent;
  using Adjoint = typename Base::Adjoint;

  friend class SO2Base<SO2<Scalar, Options>>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   SO2() : unit_complex_(Scalar(1), Scalar(0)) {}

  template <class OtherDerived>
   SO2(SO2Base<OtherDerived> const& other)
      : unit_complex_(other.unit_complex()) {}

   explicit SO2(Transformation const& R)
      : unit_complex_(Scalar(0.5) * (R(0, 0) + R(1, 1)),
                      Scalar(0.5) * (R(1, 0) - R(0, 1))) {
    assert(std::abs(R.determinant() - Scalar(1)) <= Constants<Scalar>::epsilon());
  }

   SO2(Scalar const& real, Scalar const& imag)
      : unit_complex_(real, imag) {
    Base::normalize();
  }

   explicit SO2(Vector2<Scalar> const& complex)
      : unit_complex_(complex) {
    Base::normalize();
  }

   explicit SO2(Scalar theta) {
    unit_complex_nonconst() = SO2<Scalar>::exp(theta).unit_complex();
  }

   Vector2<Scalar> const& unit_complex() const {
    return unit_complex_;
  }

 protected:
   Vector2<Scalar>& unit_complex_nonconst() { return unit_complex_; }

  Lie::Vector2<Scalar> unit_complex_;
};

template <class Scalar, int Options = 0>
using SO2Group[[deprecated]] = SO2<Scalar, Options>;

}  // namespace Lie

namespace Eigen {

template <class Scalar_, int Options>
class Map<Lie::SO2<Scalar_>, Options>
    : public Lie::SO2Base<Map<Lie::SO2<Scalar_>, Options>> {
  using Base = Lie::SO2Base<Map<Lie::SO2<Scalar_>, Options>>;

 public:
  using Scalar = Scalar_;

  using Transformation = typename Base::Transformation;
  using Point = typename Base::Point;
  using Tangent = typename Base::Tangent;
  using Adjoint = typename Base::Adjoint;

  friend class Lie::SO2Base<Map<Lie::SO2<Scalar_>, Options>>;

  EIGEN_INHERIT_ASSIGNMENT_EQUAL_OPERATOR(Map)
  using Base::operator*=;
  using Base::operator*;

  
  Map(Scalar* coeffs) : unit_complex_(coeffs) {}

  
  Map<Lie::Vector2<Scalar>, Options> const& unit_complex() const {
    return unit_complex_;
  }

 protected:
  
  Map<Lie::Vector2<Scalar>, Options>& unit_complex_nonconst() {
    return unit_complex_;
  }

  Map<Matrix<Scalar, 2, 1>, Options> unit_complex_;
};

template <class Scalar_, int Options>
class Map<Lie::SO2<Scalar_> const, Options>
    : public Lie::SO2Base<Map<Lie::SO2<Scalar_> const, Options>> {
  using Base = Lie::SO2Base<Map<Lie::SO2<Scalar_> const, Options>>;

 public:
  using Scalar = Scalar_;
  using Transformation = typename Base::Transformation;
  using Point = typename Base::Point;
  using Tangent = typename Base::Tangent;
  using Adjoint = typename Base::Adjoint;

  EIGEN_INHERIT_ASSIGNMENT_EQUAL_OPERATOR(Map)
  using Base::operator*=;
  using Base::operator*;

   Map(Scalar const* coeffs) : unit_complex_(coeffs) {}

   Map<Lie::Vector2<Scalar> const, Options> const& unit_complex()
      const {
    return unit_complex_;
  }

 protected:
  Map<Matrix<Scalar, 2, 1> const, Options> const unit_complex_;
};
}

#endif  // LIE_SO2_HPP
