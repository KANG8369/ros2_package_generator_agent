import os
import shutil
from pathlib import Path
from typing import List
from pydantic import BaseModel, Field

from langchain.chat_models import init_chat_model
from langchain_core.prompts import ChatPromptTemplate

# --------------- Structured Output Schema -----------------

class Ros2File(BaseModel):
    path: str = Field(..., description="Relative POSIX path inside package.")
    content: str = Field(..., description="Full UTF-8 text of file.")

class Ros2PackageSpec(BaseModel):
    package_name: str = Field(..., description="ROS2 package name")
    files: List[Ros2File]


# --------------- ROS2 System Prompt -----------------

ROS2_SYSTEM_PROMPT = """
You are a ROS2 (Robot Operating System 2) package generator assistant.

Goal:
Given a natural-language description from the user, you must design a
single ROS2 package and output a *complete* file set for it.

General requirements:
- Assume ROS2 Humble.
- Use ament_cmake build system.
- Use **C++ (rclcpp)** for all node source code.
- Follow standard ROS2 package structure:

  {{package_name}}/
    CMakeLists.txt
    package.xml
    src/
      <node_sources.cpp>
    include/{{package_name}}/
      <headers if needed>
    launch/
      <launch files, .py>
    config(optional)/
      <YAML params if needed>

Mandatory:
- All executables must be registered using `add_executable()` and
  `install(TARGETS ...)`.
- All dependencies must match those actually used in the node's code.

What you must output:
- The ROS2 package name.
- A list of files:
  - path
  - content

Rules:
- All files must be complete.
- No placeholders.
- No omitted code.
- Must be consistent.
"""


# --------------- Build the Generator -----------------

def build_ros2_generator():
    model = init_chat_model("gpt-5.1", temperature=0)
    structured_model = model.with_structured_output(Ros2PackageSpec)

    prompt = ChatPromptTemplate.from_messages(
        [
            ("system", ROS2_SYSTEM_PROMPT),
            ("human", "{description}"),
        ]
    )
    return prompt | structured_model


# --------------- Write Files + Zip -----------------

def materialize_ros2_package(spec: Ros2PackageSpec, output_root: Path = Path("generated_ros2_packages")) -> Path:
    pkg_dir = output_root / spec.package_name

    if pkg_dir.exists():
        shutil.rmtree(pkg_dir)
    pkg_dir.mkdir(parents=True, exist_ok=True)

    for f in spec.files:
        fp = pkg_dir / f.path
        fp.parent.mkdir(parents=True, exist_ok=True)
        fp.write_text(f.content, encoding="utf-8")

    zip_path = shutil.make_archive(str(output_root / spec.package_name), "zip",
                                   root_dir=output_root, base_dir=spec.package_name)
    return Path(zip_path)


# --------------- NEW: Process description/*.txt -----------------

def main():
    if "OPENAI_API_KEY" not in os.environ:
        raise RuntimeError("Set OPENAI_API_KEY env variable.")

    generator = build_ros2_generator()

    description_dir = Path("description")
    if not description_dir.exists():
        raise RuntimeError("❌ 'description/' folder not found.")

    txt_files = list(description_dir.glob("*.txt"))
    if not txt_files:
        raise RuntimeError("❌ No .txt files found in 'description/' folder.")

    print("=== ROS2 Package Generator Batch Mode ===")
    print(f"Found {len(txt_files)} description files.\n")

    for txt in txt_files:
        print(f"📄 Processing description: {txt.name}")

        description_text = txt.read_text(encoding="utf-8")

        # Let GPT decide package_name, but give optional hint from file name.
        spec: Ros2PackageSpec = generator.invoke(
            {"description": description_text}
        )

        print(f" → Package name: {spec.package_name}")
        print(f" → Files: {len(spec.files)}")

        zip_path = materialize_ros2_package(spec)
        print(f" ✔ Saved: {zip_path}\n")

    print("🎉 All packages generated successfully!")


if __name__ == "__main__":
    main()
