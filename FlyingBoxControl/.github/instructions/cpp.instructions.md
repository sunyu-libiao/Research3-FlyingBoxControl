---
applyTo: "**/*.{c,cpp,cc,cxx,h,hpp,hxx}"
---

# C / C++ 工程规约

本规约在编辑所有 C / C++ 源码及头文件时自动注入并生效。

---

## 1. 内存与资源安全 (RAII 铁律)
- **绝对 RAII 纪律**：严禁裸指针 `new` / `delete` 或 `malloc` / `free`；动态所有权统一使用 `std::unique_ptr`，共享所有权使用 `std::shared_ptr`。
- **视图类型优先传递**：对于只读连续内存或只读字符串传递，优先采用 `std::string_view` 与 `std::span`，消除无谓拷贝。

## 2. 现代 C++ 规范与类型安全
- **Const 正确性 (Const-Correctness)**：所有不修改成员状态的方法一律标为 `const`；局部不可变变量统一标为 `const` 或 `constexpr`。
- **严禁 C 风格强转**：类型转换必须显式使用 `static_cast`、`reinterpret_cast` 或 `dynamic_cast`，杜绝隐式切片与未定义行为。
- **显式所有权转移**：耗时对象传递采用移动语义 (`std::move`)，移动后的对象不得再次访问。

## 3. 头文件与编译解耦
- **头文件卫士**：统一使用 `#pragma once`。
- **前置声明与最小包含**：头文件中能使用前置声明（Forward Declaration）的，严禁包含完整头文件，缩短构建时长。
