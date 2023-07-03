#pragma once

#include <fmt/core.h>
#include <fmt/format.h>

template void fmt::detail::vformat_to<char>(buffer<char>& buf, basic_string_view<char> fmt,
                                            basic_format_args<FMT_BUFFER_CONTEXT(type_identity_t<char>)> args,
                                            locale_ref loc);