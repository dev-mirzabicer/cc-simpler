import os

global res
res = ""


def prpr(st):
    global res
    res += st + "\n"


def tree(path, indent=""):
    """
    Recursively prints the directory structure and file contents like the 'tree' command.

    Args:
      path: The path to the directory to start from.
      indent: The indentation string to use for subdirectories.
    """

    prpr(indent + os.path.basename(path))

    for item in os.listdir(path):
        item_path = os.path.join(path, item)
        if os.path.isdir(item_path):
            tree(item_path, indent + "  ")
        elif "tree.py" not in item_path:
            prpr(indent + "  " + item)
            try:
                with open(item_path, "r") as f:
                    prpr(indent + "    " + "---- File Content ----")
                    for line in f:
                        prpr(indent + "    " + line.strip())
                    prpr(indent + "    " + "---------------------")
            except UnicodeDecodeError:
                prpr(indent + "    " + "[Binary File - Content Not Displayed]")


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1:
        start_path = sys.argv[1]
    else:
        start_path = "."

    tree(start_path)
    with open("./tree.txt", "w") as file:
        file.write(res)
