/*
Derived from https://arne-mertz.de/2018/05/overload-build-a-variant-visitor-on-the-fly/ and
https://stackoverflow.com/a/64018031

Example usage:
```
auto visitor = overload{
        [](const Quit& q)        { std::cout << "Quit\n"; },
        [](const Move& m)        { std::cout << "Move " << m.x << " " << m.y << "\n"; },
        [](const Write& w)       { std::cout << "Write " << w.s << "\n"; },
        [](const ChangeColor& c) { std::cout << "ChangeColor " << c.r << " " << c.g << " " << c.b << "\n"; }
    };

    Message m1{Quit{}};
    Message m2{Move{1, 2}};
    Message m3{Write{"a"}};
    Message m4{ChangeColor{1, 2, 3}};
    std::visit(visitor, m1);
    std::visit(visitor, m2);
    std::visit(visitor, m3);
    std::visit(visitor, m4);
```

*/
#pragma once
template <class... Ts>
struct overload : Ts... {
    using Ts::operator()...;
};

template <class... Ts>
overload(Ts...) -> overload<Ts...>;