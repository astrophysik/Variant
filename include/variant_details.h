#pragma once

#include <cstddef>
#include <exception>
#include <functional>
#include <type_traits>
#include <utility>

template <typename... Types>
struct variant;

inline constexpr size_t variant_npos = -1;

template <typename T>
struct in_place_type_t {
};

template <typename T>
inline constexpr in_place_type_t<T> in_place_type;

template <size_t N>
struct in_place_index_t {
};

template <size_t N>
inline constexpr in_place_index_t<N> in_place_index;

template <size_t Pos, typename Variant>
struct variant_alternative;

template <size_t Pos, typename Head, typename... Tail>
struct variant_alternative<Pos, variant<Head, Tail...>> : variant_alternative<Pos - 1, variant<Tail...>> {};

template <typename Head, typename... Tail>
struct variant_alternative<0, variant<Head, Tail...>> {
  using type = Head;
};

template <typename Head, typename... Tail>
struct variant_alternative<variant_npos, variant<Head, Tail...>> {
  using type = void;
};

template <size_t Pos, typename Variant>
struct variant_alternative<Pos, const Variant> {
  using type = std::add_const_t<typename variant_alternative<Pos, Variant>::type>;
};

template <size_t Pos, typename Variant>
using variant_alternative_t = typename variant_alternative<Pos, Variant>::type;

template <typename Variant>
struct variant_size;

template <typename Variant>
struct variant_size<const Variant> : variant_size<Variant> {};

template <typename... Types>
struct variant_size<variant<Types...>> : std::integral_constant<size_t, sizeof...(Types)> {};

template <typename Variant>
inline constexpr size_t variant_size_v = variant_size<Variant>::value;

struct bad_variant_access : public std::exception {};

struct monostate {};

namespace variant_details {

/**
 * Concepts
 **/

template <typename... Types>
concept All_Trivially_Destructible = (... && std::is_trivially_destructible_v<Types>);

template <typename... Types>
concept All_Nothrow_Destructible = (... && std::is_nothrow_destructible_v<Types>);

template <typename... Types>
concept All_Trivially_Copy_Constructible = (... && std::is_trivially_copyable_v<Types>);

template <typename... Types>
concept All_Trivially_Move_Constructible = (... && std::is_trivially_move_constructible_v<Types>);

template <typename... Types>
concept All_Copy_Constructible = (... && std::is_copy_constructible_v<Types>);

template <typename... Types>
concept All_Move_Constructible = (... && std::is_move_constructible_v<Types>);

template <typename... Types>
concept All_Copy_Assignable = (... && std::is_copy_assignable_v<Types>);

template <typename... Types>
concept All_Move_Assignable = (... && std::is_move_assignable_v<Types>);

template <typename... Types>
concept All_Trivially_Copy_Assignable = (... && std::is_trivially_copy_assignable_v<Types>);

template <typename... Types>
concept All_Trivially_Move_Assignable = (... && std::is_trivially_move_assignable_v<Types>);

template <typename... Types>
concept All_Nothrow_Copy_Constructible = (... && std::is_nothrow_copy_constructible_v<Types>);

template <typename... Types>
concept All_Nothrow_Move_Constructible = (... && std::is_nothrow_move_constructible_v<Types>);

template <typename... Types>
concept All_Nothrow_Copy_Assignable = (... && std::is_nothrow_copy_assignable_v<Types>);

template <typename... Types>
concept All_Nothrow_Move_Assignable = (... && std::is_nothrow_move_assignable_v<Types>);

template <typename Variant>
concept First_Is_Nothrow_Default_Constructible =
    std::is_nothrow_default_constructible_v<variant_alternative_t<0, Variant>>;

template <typename Variant>
concept First_Is_Default_Constructible = std::is_default_constructible_v<variant_alternative_t<0, Variant>>;

template <typename T, typename... Types>
concept One_In_List = (... + std::is_same_v<T, Types>) == 1;

template <typename... Types>
concept All_Nothrow_Swappable = ((std::is_nothrow_move_constructible_v<Types> &&
                                  std::is_nothrow_swappable_v<Types>)&&...);

template <typename T>
concept Three_Way_Comparable = requires(T t) {
  t <=> t;
};

template <typename... Types>
concept All_Types_Three_Way_Comparable = (... && Three_Way_Comparable<Types>);

template<typename T>
struct helper_for_suit_type {
  static constexpr bool value = true;
};

template<typename T>
struct helper_for_suit_type<in_place_type_t<T>> {
  static constexpr bool value = false;
};

template<size_t N>
struct helper_for_suit_type<in_place_index_t<N>> {
  static constexpr bool value = false;
};

template<typename... Types>
struct helper_for_suit_type<variant<Types...>> {
  static constexpr bool value = false;
};

template <typename T>
concept Suitable_Type = helper_for_suit_type<T>::value;

template <typename T, typename T_i>
concept Necessary_Valid_Declaration = requires(T t) {
  new T_i[1]{{ std::forward<T>(t)}};
};

/**
 * Metafunctions
 **/
template <size_t N, typename T, typename Head, typename... Tail>
struct index_in_list {
  static constexpr size_t value = index_in_list<N + 1, T, Tail...>::value;
};

template <size_t N, typename T, typename... Tail>
struct index_in_list<N, T, T, Tail...> {
  static constexpr size_t value = N;
};

template <size_t N, typename... Types>
struct Nth_type;

template <size_t N, typename Head, typename... Tail>
struct Nth_type<N, Head, Tail...> : Nth_type<N - 1, Tail...> {};

template <typename Head, typename... Tail>
struct Nth_type<0, Head, Tail...> {
  using type = Head;
};

/**
 * Imaginary functions for converting constructor and operator
 **/
template <size_t Pos, typename T, typename T_i>
struct build_fun {
  static std::integral_constant<size_t, Pos> fun(T_i) requires(Necessary_Valid_Declaration<T, T_i>);
  static void fun() requires(!Necessary_Valid_Declaration<T, T_i>);
};

template <typename T, typename Variant, typename = std::make_index_sequence<variant_size_v<Variant>>>
struct build_funs;

template <typename T, typename... T_i, size_t... Ind>
struct build_funs<T, variant<T_i...>, std::index_sequence<Ind...>> : build_fun<Ind, T, T_i>... {
  using build_fun<Ind, T, T_i>::fun...;
};

template <typename T, typename Variant>
concept There_Is_A_Only_One_Im_Fun = requires() {
  build_funs<T, Variant>::fun(std::declval<T>());
};

template <typename T, typename Variant>
struct accepted_index : std::integral_constant<size_t, variant_npos> {};

template <typename T, typename Variant>
requires(There_Is_A_Only_One_Im_Fun<T, Variant>) struct accepted_index<T, Variant>
    : decltype(build_funs<T, Variant>::fun(std::declval<T>())) {};

template <typename T, typename Variant>
constexpr size_t accepted_index_v = accepted_index<T, Variant>::value;

template <typename T, typename Variant>
using accepted_type = variant_alternative_t<accepted_index_v<T, Variant>, Variant>;

/**
 * visit helper classes
 **/
template <typename Func_ptr, size_t... Dimensions>
struct multi_dim_table {
  constexpr const Func_ptr& access() const {
    return data;
  }

  Func_ptr data;
};

template <typename Func_ptr, size_t First, size_t... Rest>
struct multi_dim_table<Func_ptr, First, Rest...> {
  template <typename... Args>
  constexpr const Func_ptr& access(size_t first_id, Args... rest_id) const {
    return data[first_id].access(rest_id...);
  }

  multi_dim_table<Func_ptr, Rest...> data[First];
};

template <typename Array_type, typename Variant_tuple, typename Index_seq>
struct get_visit_table_impl;

template <typename Result_type, typename Visitor, size_t... Dimensions, typename... Variants, size_t... Indices>
struct get_visit_table_impl<multi_dim_table<Result_type (*)(Visitor, Variants...), Dimensions...>,
                            std::tuple<Variants...>, std::index_sequence<Indices...>> {

  using Next = std::remove_reference_t<typename Nth_type<sizeof...(Indices), Variants...>::type>;
  using Array_type = multi_dim_table<Result_type (*)(Visitor, Variants...), Dimensions...>;

  static constexpr Array_type apply() {
    Array_type table{};
    apply_all_alt(table, std::make_index_sequence<variant_size_v<Next>>());
    return table;
  }

  template <size_t... var_indices>
  static constexpr void apply_all_alt(Array_type& visit_table, std::index_sequence<var_indices...>) {
    (apply_single_alt<var_indices>(visit_table.data[var_indices]), ...);
  }

  template <size_t Index, typename T>
  static constexpr void apply_single_alt(T& element) {
    element = get_visit_table_impl<std::remove_reference_t<decltype(element)>, std::tuple<Variants...>,
                                   std::index_sequence<Indices..., Index>>::apply();
  }
};

template <typename Result, typename Visitor, typename... Variants, size_t... Indices>
struct get_visit_table_impl<multi_dim_table<Result (*)(Visitor, Variants...)>, std::tuple<Variants...>,
                            std::index_sequence<Indices...>> {

  using Array_type = multi_dim_table<Result (*)(Visitor&&, Variants...)>;

  decltype(auto) static constexpr visit_invoke(Visitor&& visitor, Variants... variants) {
    return std::invoke(std::forward<Visitor>(visitor), get<Indices>(std::forward<Variants>(variants))...);
  }

  static constexpr auto apply() {
    return Array_type{&visit_invoke};
  }
};

template <typename Result, typename Visitor, typename... Variants>
struct gen_visit_table {
  using Func_ptr = Result (*)(Visitor&&, Variants...);
  using Array_type = multi_dim_table<Func_ptr, variant_size_v<std::remove_reference_t<Variants>>...>;

  static constexpr auto visit_table =
      get_visit_table_impl<Array_type, std::tuple<Variants...>, std::index_sequence<>>::apply();
};

/**
 * get helper functions
 **/
template <size_t Pos, typename Variant_Base>
constexpr decltype(auto) get(Variant_Base&& v) {
  return get(in_place_index<Pos>, std::forward<Variant_Base>(v).storage);
}

template <size_t Pos, typename Variadic_Union>
constexpr decltype(auto) get(in_place_index_t<Pos>, Variadic_Union&& v) {
  return get(in_place_index<Pos - 1>, std::forward<Variadic_Union>(v).tail);
}

template <typename Variadic_Union>
constexpr decltype(auto) get(in_place_index_t<0>, Variadic_Union&& v) {
  return std::forward<Variadic_Union>(v).head.get();
}

template <size_t Pos, typename Variant>
constexpr decltype(auto) get_impl(Variant&& v) {
  return variant_details::get(in_place_index<Pos>, std::forward<Variant>(v).storage);
}

/**
 * type erased class members
 **/
template <typename Ref>
Ref ref_cast(void* ptr) {
  return static_cast<Ref>(*static_cast<std::remove_reference_t<Ref>*>(ptr));
}

template <size_t Pos, typename Variant_Base>
void erased_dtor(Variant_Base&& v) {
  std::destroy_at(std::addressof(get<Pos>(std::forward<Variant_Base>(v))));
}

template <size_t Pos, typename Variant_Base, typename... Args>
void erased_emplace(Variant_Base&& v, Args&&... args) {
  std::construct_at(std::addressof(get<Pos>(std::forward<Variant_Base>(v))), std::forward<Args>(args)...);
}

template <typename Lhs, typename Rhs>
void erased_ctor(void* lhs, void* rhs) {
  new (lhs) std::remove_reference_t<Lhs>(ref_cast<Rhs>(rhs));
}

template <typename Lhs, typename Rhs>
void erased_assign(void* lhs, void* rhs) {
  ref_cast<Lhs>(lhs) = ref_cast<Rhs>(rhs);
}

template <typename Lhs, typename Rhs>
void erased_swap(void* lhs, void* rhs) {
  using std::swap;
  swap(ref_cast<Lhs>(lhs), ref_cast<Rhs>(rhs));
}

/**
 * uninitialized is trivially destructible for any type T
 **/
template <typename T>
struct uninitialized {

  template <typename... Args>
  constexpr uninitialized(Args&&... args) {
    new (&storage) T(std::forward<Args>(args)...);
  }

  const T& get() const& {
    return *reinterpret_cast<const T*>(&storage);
  }

  T& get() & {
    return *reinterpret_cast<T*>(&storage);
  }

  T&& get() && {
    return std::move(*reinterpret_cast<T*>(&storage));
  }

  std::aligned_storage_t<sizeof(T), alignof(T)> storage;
};

template <typename T>
requires(std::is_trivially_destructible_v<T>) struct uninitialized<T> {
  template <typename... Args>
  constexpr uninitialized(Args&&... args) : storage(std::forward<Args>(args)...) {}

  constexpr const T& get() const& {
    return storage;
  }

  constexpr T& get() & {
    return storage;
  }

  constexpr T&& get() && {
    return std::move(storage);
  }

  T storage;
};


template <typename... Types>
union variadic_union {};

template <typename Head, typename... Tail>
union variadic_union<Head, Tail...> {

  constexpr variadic_union() : tail() {}

  template <typename... Args>
  constexpr variadic_union(in_place_index_t<0>, Args&&... args) : head(std::forward<Args>(args)...) {}

  template <size_t Pos, typename... Args>
  constexpr variadic_union(in_place_index_t<Pos>, Args&&... args)
      : tail(in_place_index<Pos - 1>, std::forward<Args>(args)...) {}

  uninitialized<Head> head;
  variadic_union<Tail...> tail;
};

/**
 * destructor base determined by type T and has same result of std::is_trivially_destructible<>
 * added because Clang doesn't support multiply destructors
 **/
template <typename... Types>
struct destructor_variant_base {
  template <size_t... indices>
  static constexpr void (*Dtable[])(destructor_variant_base&) = {
      &erased_dtor<indices, destructor_variant_base&>...};

  constexpr destructor_variant_base() : index(variant_npos) {}

  template <size_t Pos, typename... Args>
  constexpr destructor_variant_base(in_place_index_t<Pos>, Args&&... args)
      : storage(in_place_index<Pos>, std::forward<Args>(args)...), index(Pos) {}

  template <size_t... indices>
  constexpr void reset_impl(std::index_sequence<indices...>) {
    if (index != variant_npos) {
      Dtable<indices...>[index](*this);
    }
  }

  void reset() {
    if (index != variant_npos) {
      reset_impl(std::index_sequence_for<Types...>{});
      index = variant_npos;
    }
  }

  ~destructor_variant_base() noexcept(All_Nothrow_Destructible<Types...>) {
    reset();
  }

  variadic_union<Types...> storage;
  size_t index;
};

template <typename... Types>
requires(All_Trivially_Destructible<Types...>) struct destructor_variant_base<Types...> {
  constexpr destructor_variant_base() : index(variant_npos) {}

  template <size_t Pos, typename... Args>
  constexpr destructor_variant_base(in_place_index_t<Pos>, Args&&... args)
      : storage(in_place_index<Pos>, std::forward<Args>(args)...), index(Pos) {}

  void reset() {
    index = variant_npos;
  }

  variadic_union<Types...> storage;
  size_t index;
};

template <typename... Types>
struct variant_base : destructor_variant_base<Types...> {
  using Base = destructor_variant_base<Types...>;
  using Base::index;
  using Base::reset;
  using Base::storage;
  constexpr variant_base() noexcept(std::is_nothrow_default_constructible_v<typename Nth_type<0, Types...>::type>){}

  template <size_t Pos, typename... Args>
  constexpr variant_base(in_place_index_t<Pos>, Args&&... args)
      noexcept(std::is_nothrow_constructible_v<typename Nth_type<Pos, Types...>::type, Args...>) : Base(in_place_index<Pos>, std::forward<Args>(args)...) {}

  constexpr variant_base(const variant_base&) requires(!All_Copy_Constructible<Types...>) = delete;

  constexpr variant_base(const variant_base&) noexcept(All_Nothrow_Copy_Constructible<Types...>)
      requires(All_Trivially_Copy_Constructible<Types...>) = default;

  variant_base(const variant_base& rhs) noexcept(All_Nothrow_Copy_Constructible<Types...>) {
    if (rhs.valid_storage()) {
      static constexpr void (*Cvtable[])(void*, void*) = {&erased_ctor<Types&, const Types&>...};
      Cvtable[rhs.index](static_cast<void*>(&storage), const_cast<void*>(static_cast<const void*>(&rhs.storage)));
      index = rhs.index;
    }
  }

  variant_base(variant_base&&) requires(!All_Move_Constructible<Types...>) = delete;

  variant_base(variant_base&&) noexcept(All_Nothrow_Move_Constructible<Types...>)
      requires(All_Trivially_Move_Constructible<Types...>) = default;

  variant_base(variant_base&& rhs) noexcept(All_Nothrow_Move_Constructible<Types...>) {
    if (rhs.valid_storage()) {
      static constexpr void (*Cvtable[])(void*, void*) = {&erased_ctor<Types&, Types&&>...};
      Cvtable[rhs.index](static_cast<void*>(&storage), const_cast<void*>(static_cast<const void*>(&rhs.storage)));
      index = rhs.index;
    }
  }

  constexpr variant_base& operator=(const variant_base&)
      noexcept(All_Nothrow_Copy_Assignable<Types...>&& All_Nothrow_Copy_Constructible<Types...>)
      requires(All_Trivially_Copy_Constructible<Types...>&& All_Trivially_Copy_Assignable<Types...>&&
                   All_Trivially_Destructible<Types...>) = default;

  constexpr variant_base& operator=(const variant_base& rhs)
      noexcept(All_Nothrow_Copy_Assignable<Types...>&& All_Nothrow_Copy_Constructible<Types...>) {
    if (this == &rhs || (!rhs.valid_storage() && !valid_storage())) {
      return *this;
    }
    if (!rhs.valid_storage()) {
      reset();
      return *this;
    }
    if (index == rhs.index) {
      constexpr void (*Avtable[])(void*, void*) = {&erased_assign<Types&, const Types&>...};
      Avtable[rhs.index](static_cast<void*>(&storage),
                         const_cast<void*>(static_cast<const void*>(&rhs.storage)));
    } else {
      if constexpr (std::is_nothrow_copy_constructible_v<variant_base> ||
                    !std::is_nothrow_move_constructible_v<variant_base>) {
        reset();
        new (this) variant_base(rhs);
      } else {
        this->operator=(variant_base(rhs));
      }
    }
    return *this;
  }

  constexpr variant_base& operator=(variant_base&&)
      noexcept(All_Nothrow_Move_Constructible<Types...>&& All_Nothrow_Move_Assignable<Types...>)
      requires(All_Trivially_Move_Constructible<Types...>&& All_Trivially_Move_Assignable<Types...>&&
                   All_Trivially_Destructible<Types...>) = default;

  constexpr variant_base& operator=(variant_base&& rhs)
      noexcept(All_Nothrow_Move_Assignable<Types...>&& All_Nothrow_Move_Constructible<Types...>) {
    if (this == &rhs || (!rhs.valid_storage() && !valid_storage())) {
      return *this;
    }
    if (!rhs.valid_storage()) {
      reset();
      return *this;
    }
    if (index == rhs.index) {
      constexpr void (*Avtable[])(void*, void*) = {&erased_assign<Types&, Types&&>...};
        Avtable[rhs.index](static_cast<void*>(&storage),
                           const_cast<void*>(static_cast<const void*>(&rhs.storage)));
    } else {
      reset();
      new (this) variant_base(std::move(rhs));
    }
    return *this;
  }

  constexpr void swap(variant_base& rhs) noexcept(All_Nothrow_Swappable<Types...>) {
    if (!rhs.valid_storage() && !valid_storage()) {
      return;
    } else if (rhs.index == index) {
      constexpr void (*Svtable[])(void*, void*) = {&erased_swap<Types&, Types&>...};
      Svtable[rhs.index](static_cast<void*>(&storage), static_cast<void*>(&rhs.storage));
    } else if (!valid_storage()) {
      new (this) variant_base(std::move(rhs));
      rhs.reset();
    } else if (!rhs.valid_storage()) {
      new (&rhs) variant_base(std::move(*this));
      reset();
    } else {
      auto tmp = std::move(rhs);
      rhs = std::move(*this);
      *this = std::move(tmp);
    }
  }

  template <std::size_t I, class... Args>
  requires(I < sizeof...(Types) &&
           std::is_constructible_v<typename Nth_type<I, Types...>::type, Args...>)
      constexpr void emplace(in_place_index_t<I>, Args&&... args) {
    reset();
    erased_emplace<I>(*static_cast<Base *>(this), std::forward<Args>(args)...);
    index = I;
  }

  constexpr bool valid_storage() const noexcept {
    return Base::index != variant_npos;
  }
};
} // namespace variant_details
