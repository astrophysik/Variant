#pragma once

#include "variant_details.h"


template <size_t I, class... Types>
requires(I < sizeof...(Types)) constexpr const
    variant_alternative_t<I, variant<Types...>>& get(const variant<Types...>& v) {
  if (v.index() != I) {
    throw bad_variant_access{};
  }
  return variant_details::get_impl<I>(v);
}

template <size_t I, class... Types>
requires(I < sizeof...(Types)) constexpr variant_alternative_t<I, variant<Types...>>& get(variant<Types...>& v) {
  return const_cast<variant_alternative_t<I, variant<Types...>>&>(get<I>(std::as_const(v)));
}

template <size_t I, class... Types>
requires(I < sizeof...(Types)) constexpr const
    variant_alternative_t<I, variant<Types...>>&& get(const variant<Types...>&& v) {
  return std::move(get<I>(v));
}

template <size_t I, class... Types>
requires(I < sizeof...(Types)) constexpr variant_alternative_t<I, variant<Types...>>&& get(variant<Types...>&& v) {
  return const_cast<variant_alternative_t<I, variant<Types...>>&&>(get<I>(v));
}

template <class T, class... Types>
requires(variant_details::One_In_List<T, Types...>) constexpr const T& get(const variant<Types...>& v) {
  constexpr size_t ind = variant_details::index_in_list<0, T, Types...>::value;
  return get<ind>(v);
}

template <class T, class... Types>
requires(variant_details::One_In_List<T, Types...>) constexpr T& get(variant<Types...>& v) {
  constexpr size_t ind = variant_details::index_in_list<0, T, Types...>::value;
  return get<ind>(v);
}

template <class T, class... Types>
requires(variant_details::One_In_List<T, Types...>) constexpr T&& get(variant<Types...>&& v) {
  constexpr size_t ind = variant_details::index_in_list<0, T, Types...>::value;
  return get<ind>(v);
}

template <class T, class... Types>
requires(variant_details::One_In_List<T, Types...>) constexpr const T&& get(const variant<Types...>&& v) {
  constexpr size_t ind = variant_details::index_in_list<0, T, Types...>::value;
  return get<ind>(v);
}

template <size_t I, class... Types>
requires(I < sizeof...(Types)) constexpr std::add_pointer_t<
    const variant_alternative_t<I, variant<Types...>>> get_if(const variant<Types...>* pv) noexcept {
  if (pv != nullptr && pv->index() == I) {
    return std::addressof(variant_details::get_impl<I>(*pv));
  }
  return nullptr;
}

template <size_t I, class... Types>
requires(I < sizeof...(Types)) constexpr std::add_pointer_t<variant_alternative_t<I, variant<Types...>>> get_if(
    variant<Types...>* pv) noexcept {
  return const_cast<std::add_pointer_t<variant_alternative_t<I, variant<Types...>>>>(get_if<I>(const_cast<const variant<Types...>*>(pv)));
}

template <class T, class... Types>
requires(variant_details::One_In_List<T, Types...>) constexpr std::add_pointer_t<T> get_if(
    variant<Types...>* pv) noexcept {
  constexpr size_t I = variant_details::index_in_list<0, T, Types...>::value;
  return get_if<I>(pv);
}

template <class T, class... Types>
requires(variant_details::One_In_List<T, Types...>) constexpr std::add_pointer_t<const T> get_if(
    const variant<Types...>* pv) noexcept {
  constexpr size_t I = variant_details::index_in_list<0, T, Types...>::value;
  return get_if<I>(pv);
}

template <typename Visitor, typename... Variants>
constexpr decltype(auto) visit(Visitor&& visitor, Variants&&... variants) {
  if ((... || variants.valueless_by_exception())) {
    throw bad_variant_access{};
  }
  using Result = decltype(std::forward<Visitor>(visitor)(get<0>(std::forward<Variants>(variants))...));
  constexpr auto& visit_table = variant_details::gen_visit_table<Result, Visitor&&, Variants&&...>::visit_table;
  auto func = visit_table.access(variants.index()...);
  return (*func)(std::forward<Visitor>(visitor), std::forward<Variants>(variants)...);
}

template <typename Result, typename Visitor, typename... Variants>
constexpr Result visit(Visitor&& visitor, Variants&&... variants) {
  if ((... || variants.valueless_by_exception())) {
    throw bad_variant_access{};
  }
  constexpr auto& visit_table = variant_details::gen_visit_table<Result, Visitor&&, Variants&&...>::visit_table;
  auto func = visit_table.access(variants.index()...);
  return (*func)(std::forward<Visitor>(visitor), std::forward<Variants>(variants)...);
}

template< class T, class... Types>
constexpr bool holds_alternative(const variant<Types...>& v) noexcept {
  return v.index() == variant_details::index_in_list<0, T, Types...>::value;
}
