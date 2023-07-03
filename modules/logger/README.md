# Logger Tool{#logger}

This module should be used if you need to log something in your module. The direct write to std::cout has some limitations that are solved by using a logger:
- The logger writes the output to multiple locations if needed (e.g. the console, a file and the ui)
- The logger supports multiple log levels and the outputs can be filtered by these
- The logger objects are cached internally and will be reused. This means you should have the logger as a member in your class, but can also create a new logger everytime if it's necessary (e.g. a static context)

Usage example:
```cpp
// Create the logger object.
auto logger = luhsooccer::logger::Logger("your_module_name");

// Log a debug event using a formatted message
LOG_DEBUG(logger, "Some messages that prints a number: {}", 10);


// Log an error
LOG_ERORR(logger, "Some bad error");
```

To allow custom objects to be formatted by the logger, you just need to overwrite the `<<` stream operator for your type.
