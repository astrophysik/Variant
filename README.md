# Variant

The standart class template Variant represents a type-safe union. An instance of variant at any given time either holds a value of one of its alternative types, or in the case of error - no value (this state is hard to achieve, see valueless_by_exception). 
https://en.cppreference.com/w/cpp/utility/variant


The implementation is written in c++20 and meets all the requirements of the standard using the concepts and three way comparison.
