import sys
import ntpath

from pyparsing import Word, alphas

def parse_file(filename):
    rob_name = ""
    link = []
    joint = []

    # ===== File Extension Check =====
    file = ntpath.basename(filename)
    if file.strip().split('.')[1] != "rob":
        raise ValueError("[ ERROR ] | Invalid file type. Use '.rob' config files")
    # ================================

    # ===== File Extension Check =====
    isNamed = False

    with open(filename) as f:
        for line in f:

            # Empty line
            if not line:    
                continue

            startChar = line[0]

            match startChar:
                case ('#'): # Comments
                    continue
                case ('>'): # Robot Name
                    if isNamed:
                        raise ValueError("[ ERROR ] | The robot can only be named once.")
                    
                    rob_name = line[1:].strip()
                    isNamed = True
                case ('\\'): # Link/Joint
                    command = line[1:].strip().split()

                    match command[0]:
                        case ("link"):  # Parse Links
                            print("Link")
                        case ("joint"): # Parse Joints
                            print("Joint")

    # ================================


if __name__ == "__main__":
    in_file = sys.argv[1]
    parse_file(in_file)
