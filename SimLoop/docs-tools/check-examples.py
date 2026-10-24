"""Keep the tutorial's complete Java example aligned with the test that compiles it."""

from pathlib import Path
import argparse
import difflib
import re
import sys


module = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--document", type=Path, default=module / "docs" / "first-test.md")
args = parser.parse_args()
source_path = module / "src/test/java/org/horizon36596/simloop/example/SlideReachesTargetExampleTest.java"
source = source_path.read_text(encoding="utf-8")
document = args.document.read_text(encoding="utf-8")
match = re.search(r"^```java\n(.*?)\n```$", document, flags=re.M | re.S)
if match is None:
    sys.exit("first-test.md has no complete Java example.")

# The guide omits only the class-level Javadoc describing where the example is published.
source = re.sub(
    r"/\*\*.*?\*/\s*(?=class SlideReachesTargetExampleTest\b)", "", source, count=1, flags=re.S
).strip()
example = match.group(1).strip()
if example != source:
    diff = difflib.unified_diff(
        source.splitlines(keepends=True),
        example.splitlines(keepends=True),
        fromfile="SlideReachesTargetExampleTest.java",
        tofile="first-test.md Java block",
    )
    sys.stderr.write("The tutorial differs from its compiled example. Update both files.\n")
    sys.stderr.writelines(diff)
    sys.exit(1)
print("first-test.md matches its compiled Java example.")
