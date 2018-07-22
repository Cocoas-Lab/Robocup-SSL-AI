#ifndef AI_UTIL_MATH_TO_VECTOR_HPP_
#define AI_UTIL_MATH_TO_VECTOR_HPP_

#include <type_traits>

#include <Eigen/Dense>

#include "ai/model/command.hpp"

namespace ai {
namespace util {
namespace math {

/// @brief     メンバ関数x(), y()を持つオブジェクトを2次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Matrix<共通の型, 2, 1>{obj.x(), obj.y()}
template <class T>
inline auto position(const T& _obj)
    -> Eigen::Matrix<std::common_type_t<decltype(_obj.x()), decltype(_obj.y())>, 2, 1> {
  return {_obj.x(), _obj.y()};
}

/// @brief     position_tを2次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Vector2d{obj.x, obj.y}
inline Eigen::Vector2d position(const model::command::position& _obj) {
  return {_obj.x, _obj.y};
}

/// @brief     メンバ関数x(), y(), theta()を持つオブジェクトを3次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Matrix<共通の型, 3, 1>{obj.x(), obj.y(), obj.theta()}
template <class T>
inline auto position3d(const T& _obj) -> Eigen::Matrix<
    std::common_type_t<decltype(_obj.x()), decltype(_obj.y()), decltype(_obj.theta())>, 3, 1> {
  return {_obj.x(), _obj.y(), _obj.theta()};
}

/// @brief     position_tを3次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Vector2d{obj.x, obj.y, obj.theta}
inline Eigen::Vector3d position3d(const model::command::position& _obj) {
  return {_obj.x, _obj.y, _obj.theta};
}

/// @brief     メンバ関数vx(), vy()を持つオブジェクトを2次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Matrix<共通の型, 2, 1>{obj.vx(), obj.vy()}
template <class T>
inline auto velocity(const T& _obj)
    -> Eigen::Matrix<std::common_type_t<decltype(_obj.vx()), decltype(_obj.vy())>, 2, 1> {
  return {_obj.vx(), _obj.vy()};
}

/// @brief     velocity_tを2次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Vector2d{obj.vx, obj.vy}
inline Eigen::Vector2d velocity(const model::command::velocity& _obj) {
  return {_obj.vx, _obj.vy};
}

/// @brief     メンバ関数vx(), vy(), omega()を持つオブジェクトを3次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Matrix<共通の型, 3, 1>{obj.vx(), obj.vy(), obj.omega()}
template <class T>
inline auto velocity3d(const T& _obj) -> Eigen::Matrix<
    std::common_type_t<decltype(_obj.vx()), decltype(_obj.vy()), decltype(_obj.omega())>, 3,
    1> {
  return {_obj.vx(), _obj.vy(), _obj.omega()};
}

/// @brief     velocity_tを3次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    eigen::vector2d{obj.vx, obj.vy, obj.omega}
inline Eigen::Vector3d velocity3d(const model::command::velocity& _obj) {
  return {_obj.vx, _obj.vy, _obj.omega};
}

/// @brief     メンバ関数ax(), ay()を持つオブジェクトを2次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Matrix<共通の型, 2, 1>{obj.ax(), obj.ay()}
template <class T>
inline auto acceleration(const T& _obj)
    -> Eigen::Matrix<std::common_type_t<decltype(_obj.ax()), decltype(_obj.ay())>, 2, 1> {
  return {_obj.ax(), _obj.ay()};
}

/// @brief     acceleration_tを2次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Vector2d{obj.ax, obj.ay}
inline Eigen::Vector2d acceleration(const model::command::acceleration& _obj) {
  return {_obj.ax, _obj.ay};
}

/// @brief     メンバ関数ax(), ay(), alpha()を持つオブジェクトを3次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Matrix<共通の型, 3, 1>{obj.ax(), obj.ay(), obj.alpha()}
template <class T>
inline auto acceleration3d(const T& _obj) -> Eigen::Matrix<
    std::common_type_t<decltype(_obj.ax()), decltype(_obj.ay()), decltype(_obj.alpha())>, 3,
    1> {
  return {_obj.ax(), _obj.ay(), _obj.alpha()};
}

/// @brief     acceleration_tを3次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Vector2d{obj.ax, obj.ay, obj.alpha}
inline Eigen::Vector3d acceleration3d(const model::command::acceleration& _obj) {
  return {_obj.ax, _obj.ay, _obj.alpha};
}

} // namespace math
} // namespace util
} // namespace ai

#endif // AI_SERVER_UTIL_MATH_TO_VECTOR_H
