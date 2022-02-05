#pragma once

#include <compare>

#include "variant_functions.h"

template<typename... Types>
struct variant : private variant_details::variant_base<Types...> {
  using Base = variant_details::variant_base<Types...>;
  constexpr variant() noexcept(variant_details::First_Is_Nothrow_Default_Constructible<variant>)
      requires(variant_details::First_Is_Default_Constructible<variant>)
      : Base(in_place_index<0>) {}
  variant(const variant&) noexcept(variant_details::All_Nothrow_Copy_Constructible<Types...>)
      requires(variant_details::All_Copy_Constructible<Types...>) = default;
  variant(variant&&) noexcept(variant_details::All_Nothrow_Move_Constructible<Types...>)
      requires(variant_details::All_Move_Constructible<Types...>) = default;

  template <typename T>
  requires(sizeof...(Types) > 0 && variant_details::Suitable_Type<T> &&
      variant_details::One_In_List<
          variant_details::accepted_type<T, variant>,
          Types...>) constexpr variant(T&& t)
      noexcept(std::is_nothrow_constructible_v<variant_details::accepted_type<T, variant>, T&&>)
      : Base(in_place_index<
                 variant_details::index_in_list<0, variant_details::accepted_type<T, variant>, Types...>::value>,
             std::forward<T>(t)) {}

  template <class T, class... Args>
  requires(variant_details::One_In_List<T, Types...>&&
               std::is_constructible_v<T, Args...>) constexpr explicit variant(in_place_type_t<T>, Args&&... args)
      : Base(in_place_index<variant_details::index_in_list<0, T, Types...>::value>, std::forward<Args>(args)...) {}

  template <class T, class U, class... Args>
  constexpr explicit variant(in_place_type_t<T>, std::initializer_list<U> il, Args&&... args)
      : variant(in_place_index<variant_details::index_in_list<0, T, Types...>::value>, il,
                std::forward<Args>(args)...) {}

  template <std::size_t Pos, class... Args>
  requires(Pos < sizeof...(Types) &&
           std::is_constructible_v<variant_alternative_t<Pos, variant>,Args...>)
      constexpr explicit variant(in_place_index_t<Pos>, Args&&... args)
      : Base(in_place_index<Pos>, std::forward<Args>(args)...) {}

  template <std::size_t Pos, class U, class... Args>
  constexpr explicit variant(in_place_index_t<Pos>, std::initializer_list<U> il, Args&&... args)
      : Base(in_place_index<Pos>, il, std::forward<Args>(args)...) {}

  constexpr variant&
  operator=(const variant& rhs) noexcept(variant_details::All_Nothrow_Copy_Constructible<Types...>&&
                                             variant_details::All_Nothrow_Copy_Assignable<Types...>)
      requires(variant_details::All_Copy_Assignable<Types...>&&
                   variant_details::All_Copy_Constructible<Types...>) = default;

  constexpr variant& operator=(variant&& rhs) noexcept(variant_details::All_Nothrow_Move_Constructible<Types...>&&
                                                           variant_details::All_Nothrow_Move_Assignable<Types...>)
      requires(variant_details::All_Move_Assignable<Types...>&&
                   variant_details::All_Move_Constructible<Types...>) = default;

  template <typename T>
  requires(variant_details::One_In_List<variant_details::accepted_type<T&&, variant>, Types...> &&
           !std::is_same_v<std::remove_cvref<T>, variant> &&
           std::is_assignable_v<variant_details::accepted_type<T&&, variant>&, T> &&
           std::is_constructible_v<variant_details::accepted_type<T&&, variant>, T>) constexpr variant&
  operator=(T&& t) noexcept(
      std::is_nothrow_assignable_v<variant_details::accepted_type<T&&, variant>&, T&&>&&
          std::is_nothrow_constructible_v<variant_details::accepted_type<T&&, variant>, T&&>) {
    constexpr size_t ind = variant_details::accepted_index_v<T, variant>;
    using Type = variant_alternative_t<ind, variant>;
    if (ind == index()) {
      get<ind>(*this) = std::forward<T>(t);
    } else if (std::is_nothrow_constructible_v<Type, T> || !std::is_nothrow_move_constructible_v<Type>) {
      emplace<ind>(std::forward<T>(t));
    } else {
      emplace<ind>(Type(std::forward<T>(t)));
    }
    return *this;
  }

  template <class T, class... Args>
  requires(std::is_constructible_v<T, Args...>&& variant_details::One_In_List<T, Types...>) constexpr T& emplace(
      Args&&... args) {
    return emplace<variant_details::index_in_list<0, T, Types...>::value>(std::forward<Args>(args)...);
  }

  template <class T, class U, class... Args>
  requires(std::is_constructible_v<T, std::initializer_list<U>&, Args...>&&
               variant_details::One_In_List<T, Types...>) constexpr T& emplace(std::initializer_list<U> il,
                                                                               Args&&... args) {
    return emplace<variant_details::index_in_list<0, T, Types...>::value>(il, std::forward<Args>(args)...);
  }

  template <std::size_t I, class... Args>
  requires(I < sizeof...(Types) &&
           std::is_constructible_v<variant_alternative_t<I, variant>,Args...>)
      constexpr variant_alternative_t<I, variant>& emplace(Args&&... args) {
    Base::emplace(in_place_index<I>, std::forward<Args>(args)...);
    return get<I>(*this);
  }

  template <size_t I, class U, class... Args>
  requires(I < sizeof...(Types) &&
           std::is_constructible_v<
               variant_alternative_t<I, variant>, std::initializer_list<U>&,
               Args...>) constexpr variant_alternative_t<I, variant>& emplace(std::initializer_list<U> il,
                                                                              Args&&... args) {
    return emplace<I>(il, std::forward<Args>(args)...);
  }

  constexpr size_t index() const noexcept {
    return Base::index;
  }

  constexpr bool valueless_by_exception() const noexcept {
    return index() == variant_npos;
  }

  constexpr void swap(variant& rhs) noexcept(variant_details::All_Nothrow_Swappable<Types...>) {
    Base::swap(rhs);
  }

  template <size_t Pos, typename Variant>
  friend constexpr decltype(auto) variant_details::get_impl(Variant&& v);

  friend constexpr decltype(auto) operator<=>(const variant& lhs, const variant& rhs) requires(variant_details::All_Types_Three_Way_Comparable<Types...>) {
    if (lhs.valueless_by_exception() && rhs.valueless_by_exception()) {
      return std::strong_ordering::equal;
    } else if (lhs.valueless_by_exception()) {
      return std::strong_ordering::less;
    } else if (rhs.valueless_by_exception()) {
      return std::strong_ordering::greater;
    } else if (lhs.index() != rhs.index()) {
      return lhs.index() <=> rhs.index();
    } else {
      return visit([&lhs](auto&& rhs_value) mutable {
        using Type = std::remove_cvref<decltype(rhs_value)>;
        constexpr size_t N = variant_details::index_in_list<0, Type, Types...>::value;
        auto& lhs_value = get<N>(lhs);
        auto result = lhs_value <=> rhs_value;
        return result;
      });
    }
  }

#define VARIANT_OPERATOR_TEMPLATE(__OP) \
      friend constexpr bool operator __OP(const variant& lhs, const variant& rhs) { \
        bool ret = true;                          \
        if (rhs.index() == variant_npos) {        \
          return  (lhs.index() + 1) __OP (rhs.index() + 1);                                             \
        }                                          \
        visit([&ret, &lhs] (auto&& rhs_value) mutable {                                       \
              using Type = std::remove_cvref_t<decltype(rhs_value)>;                    \
              constexpr size_t rhs_index = variant_details::index_in_list<0, Type, Types...>::value;      \
                if (lhs.index() == rhs_index) { \
                    auto &lhs_value = get<rhs_index>(lhs);	\
                    ret = lhs_value __OP rhs_value; \
                } else { \
                ret = (lhs.index() + 1) __OP(rhs_index + 1); \
                }                                                            \
        }, rhs); \
        return ret; \
      }


  VARIANT_OPERATOR_TEMPLATE(<)
  VARIANT_OPERATOR_TEMPLATE(<=)
  VARIANT_OPERATOR_TEMPLATE(==)
  VARIANT_OPERATOR_TEMPLATE(!=)
  VARIANT_OPERATOR_TEMPLATE(>=)
  VARIANT_OPERATOR_TEMPLATE(>)

#undef VARIANT_OPERATOR_TEMPLATE
};
