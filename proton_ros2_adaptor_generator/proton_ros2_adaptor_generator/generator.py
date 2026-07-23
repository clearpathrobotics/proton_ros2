#!/usr/bin/env python3

# Copyright 2026 Rockwell Automation Technologies, Inc., All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Proton ROS 2 Adaptor Generator

Generates pluginlib adaptor packages from YAML configuration files.

Usage:
    proton_ros2_adaptor_generator generate --config CONFIG_FILE --output OUTPUT_DIR [options]
"""

import argparse
import sys
from pathlib import Path

from jinja2 import Environment, PackageLoader, select_autoescape

from .config import AdaptorConfig, MessageBinding, PackageConfig


def create_jinja_env() -> Environment:
    """Create Jinja2 environment with template loader."""
    return Environment(
        loader=PackageLoader("proton_ros2_adaptor_generator", "resources"),
        autoescape=select_autoescape(),
        trim_blocks=True,
        lstrip_blocks=True,
        keep_trailing_newline=True,
    )


def generate_message_adaptor(env: Environment, binding: MessageBinding, output_dir: Path) -> None:
    """Generate a single message adaptor C++ file."""
    template = env.get_template("adaptor.cpp.jinja")

    content = template.render(binding=binding)

    src_dir = output_dir / "src"
    src_dir.mkdir(parents=True, exist_ok=True)

    output_file = src_dir / binding.source_file
    output_file.write_text(content)
    print(f"  Generated: {output_file.relative_to(output_dir)}")


def generate_cmakelists(
    env: Environment,
    package_config: PackageConfig,
    adaptor_config: AdaptorConfig,
    output_dir: Path,
) -> None:
    """Generate CMakeLists.txt for the adaptor package."""
    template = env.get_template("CMakeLists.txt.jinja")

    content = template.render(
        package_name=package_config.package_name,
        ros_msg_dependencies=sorted(adaptor_config.ros_msg_dependencies),
        message_adaptors=adaptor_config.messages,
    )

    output_file = output_dir / "CMakeLists.txt"
    output_file.write_text(content)
    print(f"Generated: {output_file.relative_to(output_dir)}")


def generate_package_xml(
    env: Environment,
    package_config: PackageConfig,
    adaptor_config: AdaptorConfig,
    output_dir: Path,
) -> None:
    """Generate package.xml for the adaptor package."""
    template = env.get_template("package.xml.jinja")

    content = template.render(
        package_name=package_config.package_name,
        project_name=package_config.project_name,
        version=package_config.version,
        maintainer_name=package_config.maintainer_name,
        maintainer_email=package_config.maintainer_email,
        license=package_config.license,
        ros_msg_dependencies=sorted(adaptor_config.ros_msg_dependencies),
    )

    output_file = output_dir / "package.xml"
    output_file.write_text(content)
    print(f"  Generated: {output_file.relative_to(output_dir)}")


def generate_plugins_xml(
    env: Environment,
    package_config: PackageConfig,
    adaptor_config: AdaptorConfig,
    output_dir: Path,
) -> None:
    """Generate plugins.xml for pluginlib export."""
    template = env.get_template("plugins.xml.jinja")

    content = template.render(
        package_name=package_config.package_name,
        message_adaptors=adaptor_config.messages,
    )

    output_file = output_dir / "plugins.xml"
    output_file.write_text(content)
    print(f"Generated: {output_file.relative_to(output_dir)}")


def generate_package(
    config_path: Path,
    output_dir: Path,
    package_name: str,
    project_name: str,
    maintainer_name: str = "Unknown",
    maintainer_email: str = "unknown@example.com",
) -> int:
    """Generate a complete adaptor package from configuration."""
    print(f"Loading configuration from: {config_path}")

    # Load and validate configuration
    adaptor_config = AdaptorConfig.from_yaml(config_path)
    errors = adaptor_config.validate()
    if errors:
        print("Configuration errors:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        return 1

    package_config = PackageConfig(
        package_name=package_name,
        project_name=project_name,
        maintainer_name=maintainer_name,
        maintainer_email=maintainer_email,
    )

    # Create output directory
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"Generating package: {package_name}")
    print(f"Output directory: {output_dir}")

    # Create Jinja environment
    env = create_jinja_env()

    # Generate adaptor source files
    print(f"\nGenerating {len(adaptor_config.messages)} message adaptors:")
    for binding in adaptor_config.messages:
        generate_message_adaptor(env, binding, output_dir)

    # Generate package infrastructure
    print("\nGenerating package files:")
    generate_cmakelists(env, package_config, adaptor_config, output_dir)
    generate_package_xml(env, package_config, adaptor_config, output_dir)
    generate_plugins_xml(env, package_config, adaptor_config, output_dir)

    print(f"\nSuccessfully generated {package_name}")
    print(f"  Messages: {len(adaptor_config.messages)}")
    print(f"  ROS dependencies: {', '.join(sorted(adaptor_config.ros_msg_dependencies))}")

    return 0


def main() -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(
        description="Generate proton_ros2 adaptor packages from YAML configuration",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    # generate command
    gen_parser = subparsers.add_parser("generate", help="Generate an adaptor package")
    gen_parser.add_argument(
        "--config",
        "-c",
        type=Path,
        required=True,
        help="Path to YAML configuration file",
    )
    gen_parser.add_argument(
        "--output",
        "-o",
        type=Path,
        required=True,
        help="Output directory for generated package",
    )
    gen_parser.add_argument(
        "--package-name",
        "-p",
        type=str,
        required=True,
        help="Name of the generated ROS package",
    )
    gen_parser.add_argument(
        "--project-name",
        type=str,
        required=True,
        help="Project name for description (e.g., 'A300')",
    )
    gen_parser.add_argument(
        "--maintainer-name",
        type=str,
        default="Unknown",
        help="Package maintainer name",
    )
    gen_parser.add_argument(
        "--maintainer-email",
        type=str,
        default="unknown@example.com",
        help="Package maintainer email",
    )

    args = parser.parse_args()

    if args.command == "generate":
        return generate_package(
            config_path=args.config,
            output_dir=args.output,
            package_name=args.package_name,
            project_name=args.project_name,
            maintainer_name=args.maintainer_name,
            maintainer_email=args.maintainer_email,
        )

    return 0


if __name__ == "__main__":
    sys.exit(main())
