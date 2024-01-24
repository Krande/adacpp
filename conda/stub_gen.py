import argparse
import importlib
import logging
import pathlib
from typing import Union

import pybind11_stubgen

logger = logging.getLogger(__name__)


def run(
    parser: pybind11_stubgen.IParser,
    printer: pybind11_stubgen.Printer,
    module_name: str,
    out_dir: pathlib.Path,
    sub_dir: Union[pathlib.Path, None],
    dry_run: bool,
):
    module = parser.handle_module(
        pybind11_stubgen.QualifiedName.from_str(module_name),
        importlib.import_module(module_name),
    )

    parser.finalize()

    if module is None:
        raise RuntimeError(f"Can't parse {module_name}")

    if dry_run:
        return

    writer = pybind11_stubgen.Writer()

    out_dir.mkdir(exist_ok=True)
    logger.info(f"Writing stubs to {out_dir}")
    writer.write_module(module, printer, to=out_dir, sub_dir=sub_dir)


def main():
    module_name = "adacpp"
    pyi_dest_dir = pathlib.Path(__import__(module_name).__file__).parent.parent
    # pyi_dest_dir = ".stubs"

    args = argparse.Namespace(
        module_name=module_name,
        ignore_all_errors=None,
        ignore_invalid_identifiers=None,
        ignore_invalid_expressions=None,
        ignore_unresolved_names=None,
        print_invalid_expressions_as_is=False,
        output_dir=pyi_dest_dir,
        root_suffix=None,
        # set_ignored_invalid_identifiers=None,
        # set_ignored_invalid_expressions=None,
        # set_ignored_unresolved_names=None,
        exit_code=False,
        numpy_array_wrap_with_annotated_fixed_size=True,
        numpy_array_remove_parameters=True,
        numpy_array_wrap_with_annotated=True,
        dry_run=False,
    )
    # shutil.copytree('stubs', dummy_lib, dirs_exist_ok=True)
    logging.basicConfig(
        level=logging.INFO,
        format="%(name)s - [%(levelname)7s] %(message)s",
    )

    parser = pybind11_stubgen.stub_parser_from_args(args)
    printer = pybind11_stubgen.Printer(
        invalid_expr_as_ellipses=not args.print_invalid_expressions_as_is
    )

    out_dir = pathlib.Path(args.output_dir)
    out_dir.mkdir(exist_ok=True)

    if args.root_suffix is None:
        sub_dir = None
    else:
        sub_dir = pathlib.Path(f"{args.module_name}{args.root_suffix}")
    try:
        run(
            parser,
            printer,
            args.module_name,
            out_dir,
            sub_dir=sub_dir,
            dry_run=args.dry_run,
        )
    except BaseException as e:
        logger.error(f"generator error -> {e}")


if __name__ == "__main__":
    main()
