import inspect
import _baguette_py
import typing
import re

MODULE_NAME = "_baguette_py"
module = _baguette_py


def generate_nanobind_method_stubs(
    output_text: typing.List[str],
    base_name: str,
    name: str,
    method,
    callback,
    function=False,
    from_class=True,
):
    doc = inspect.getdoc(method)
    if doc is None:
        raise Exception(f"Nanobind method/function {name} has no docstring")

    docs: typing.List[str]
    if "\n" in doc:
        docs = doc.split("\n")
    else:
        docs = [doc]

    prefix = ""

    if from_class:
        prefix = "    "

    for doc in docs:
        if len(docs) > 1:
            output_text.append(f"{prefix}@overload")

        matches = re.search(r"_baguette_py\.([a-zA-Z0-9_]*)", doc)
        if matches is not None:
            for group in matches.groups():
                if base_name != group:
                    callback(group)

        doc_clean = re.sub(r"numpy.ndarray\[[^\]]*]", "numpy.typing.NDArray[Any]", doc)
        doc_clean = doc_clean.replace(f"{MODULE_NAME}.", "")
        if function and from_class:
            output_text.append(f"{prefix}@staticmethod")
        output_text.append(f"{prefix}def {doc_clean}:")
        output_text.append(f"{prefix}    '''")
        output_text.append(f"{prefix}    {doc}")
        output_text.append(f"{prefix}    '''")
        output_text.append(f"{prefix}    ...")
        output_text.append("")


def generate_method_stubs(output: typing.List[str], name: str, method):
    pass


def generate_slot_wrapper_stubs(output: typing.List[str], name: str, wrapper):
    pass


def generate_nanobind_property_stubs(output: typing.List[str], name: str, property):
    doc = inspect.getdoc(property)

    if doc is None:
        raise Exception(f"Nanobind property {name} has no docstring")

    doc_clean = re.sub(r"numpy.ndarray\[[^\]]*]", "numpy.typing.NDArray[Any]", doc)
    doc_clean = doc_clean.replace(f"{MODULE_NAME}.", "")
    doc_clean = doc_clean.replace("(self)", f"{name}(self)")
    output.append("    @property")
    output.append(f"    def {doc_clean}:")
    output.append("         '''")
    output.append(f"         {doc_clean}")
    output.append("         '''")
    output.append("         ...")
    output.append("")


def generate_nanobind_enum_stubs(output: typing.List[str], enum: dict):
    for key in enum.keys():
        var = enum[key]
        var_name = var[0]
        output.append(f"    {var_name} = ...")


def generate_class_stubs(
    generated_classes: typing.List[str], output: typing.List[str], name: str, object
):
    if name in generated_classes:
        return

    members = inspect.getmembers(object)

    doc = inspect.getdoc(object)

    mro = inspect.getmro(object)

    output_text = []

    is_enum = False

    for member in members:
        if "@entries" in member[0]:
            is_enum = True
            break

    if len(mro) > 2:
        base_classes = ""

        for base in mro[1:-1]:
            generate_class_stubs(generated_classes, output, base.__name__, base)
            base_classes += base.__name__ + ", "
        base_classes = base_classes.removesuffix(", ")
        output_text.append(f"class {name}({base_classes}):")

    elif is_enum:
        output_text.append(f"class {name}(Enum):")
    else:
        output_text.append(f"class {name}:")

    if doc is not None:
        output_text.append("'''")
        output_text.append(doc)
        output_text.append("'''")
    changed = False

    for member in members:
        if "nanobind.nb_method" in str(member[1]):
            changed = True
            generate_nanobind_method_stubs(
                output_text,
                name,
                member[0],
                member[1],
                lambda member_before: generate_class_stubs(
                    generated_classes,
                    output,
                    member_before,
                    getattr(module, member_before),
                ),
            )
        elif "property object" in str(member[1]):
            changed = True
            generate_nanobind_property_stubs(output_text, member[0], member[1])
        elif "nanobind.nb_func" in str(member[1]):
            changed = True
            generate_nanobind_method_stubs(
                output_text,
                name,
                member[0],
                member[1],
                lambda member_before: generate_class_stubs(
                    generated_classes,
                    output,
                    member_before,
                    getattr(module, member_before),
                ),
                True,
            )
        elif "@entries" in member[0]:
            changed = True
            generate_nanobind_enum_stubs(output_text, member[1])
        elif "slot wrapper" in str(member[1]):
            continue
            # changed = True
            # generate_slot_wrapper_stubs(output_text, member[0], member[1])
        elif "method" in str(member[1]):
            continue
            # changed = True
            # generate_method_stubs(output_text, member[0], member[1])
        elif name in str(member[1]):
            # Enum instances are already generated
            continue
        elif member[0].startswith("__"):
            continue
        else:
            output_text.append(f"    {member[0]}: {type(member[1]).__name__}")
            output_text.append("")
            # print(f"Skipping {member[0]} with type: {member[1]}")
            pass

    if not changed:
        output_text.append("    pass")

    generated_classes.append(name)
    output_text.append("")

    for line in output_text:
        output.append(line)


if __name__ == "__main__":
    output = []

    doc = inspect.getdoc(module)

    custom_defines = []

    output.append("'''")
    if doc is not None:
        for line in doc.split("\n"):
            if line.startswith("%%"):
                custom_defines.append(line.removeprefix("%%"))
    output.append(doc)
    output.append("'''")

    output.append(
        "from typing import overload, Callable, Optional, Union, Any, TypeVar, Type"
    )
    output.append("from enum import Enum")
    output.append("import numpy")
    output.append("import numpy.typing")
    output.append("import datetime")
    output.append("")
    output.append("#### Baguette custom code starts here ####")
    output.append("")
    for define in custom_defines:
        output.append(define)
    output.append("")
    output.append("#### Baguette custom code ends here ####")
    output.append("")
    print("Generating stubs for baguette module")

    members = inspect.getmembers(module)

    generated_classes = []

    for member in members:
        if inspect.isclass(member[1]):
            generate_class_stubs(generated_classes, output, member[0], member[1])
        elif "nanobind.nb_func" in str(member[1]):
            output_text = []
            generate_nanobind_method_stubs(
                output_text,
                "",
                member[0],
                member[1],
                lambda member_before: generate_class_stubs(
                    generated_classes,
                    output,
                    member_before,
                    getattr(module, member_before),
                ),
                True,
                False,
            )
            for line in output_text:
                output.append(line)
        elif member[0].startswith("__"):
            continue
        elif inspect.ismodule(member[1]):
            # Skip modules for now
            print(f"Skipping module: {member[0]}. Currently not supported")
            continue
        else:
            print(f"Skipping {member[0]} with type: {member[1]}")
            raise Exception(f"Unknown member: {member[0]} with type: {member[1]}")

    with open("_baguette_py.pyi", "w") as f:
        in_docstring = False
        for line in output:
            if "'''" in line:
                in_docstring = not in_docstring
            if not in_docstring:
                # Various fixes
                line = line.replace("[0. 0. 0.]", "numpy.asarray([0.0, 0.0, 0.0])")
                line = line.replace(
                    "TimePoint = 0.000000000", "TimePoint = TimePoint(0.0)"
                )
                line = line.replace(
                    "Duration = 0.000000000", "Duration = Duration(0.0)"
                )
                line = line.replace("(0.000, 0.000, 0.000°)@World", '""')
                line = line.replace("0:00:00", "0")

                # Hot fix:
                line = line.replace(
                    "def getInstance() -> :", 'def getInstance() -> "NativeBaguette":'
                )
            f.write(line + "\n")
