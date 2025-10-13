import os
import re

def generate_header_guard(file_path):
    """Generate a header guard name based on the file name."""
    file_name = os.path.basename(file_path)
    header_guard = re.sub(r'[^a-zA-Z0-9]', '_', file_name.upper())
    return f"{header_guard}"

def update_header_guards(folder_path):
    """Update header guards for all .hpp files in the folder."""
    for root, _, files in os.walk(folder_path):
        for file in files:
            if file.endswith(".hpp"):
                file_path = os.path.join(root, file)
                with open(file_path, 'r') as f:
                    lines = f.readlines()

                # Identify existing header guards
                ifndef_index = None
                define_index = None
                endif_index = None

                for i, line in enumerate(lines):
                    if line.startswith('#ifndef'):
                        ifndef_index = i
                    elif line.startswith('#define') and ifndef_index is not None:
                        define_index = i
                    elif line.startswith('#endif') and define_index is not None:
                        endif_index = i
                        break

                # Generate new header guard
                new_guard = generate_header_guard(file_path)

                # Remove old guards if they exist
                if ifndef_index is not None and define_index is not None and endif_index is not None:
                    lines = lines[:ifndef_index] + lines[define_index + 1:endif_index] + lines[endif_index + 1:]

                # Add new guards
                updated_lines = [
                    f"#ifndef {new_guard}\n",
                    f"#define {new_guard}\n"
                ] + lines + [
                    f"#endif // {new_guard}\n"
                ]

                # Write updated file
                with open(file_path, 'w') as f:
                    f.writelines(updated_lines)
                print(f"Updated header guard for: {file_path}")

if __name__ == "__main__":
    print("Please make sure the changes can be reverted easily. Use at your own risk.")
    folder = input("Enter the folder path containing .hpp files: ").strip()
    if os.path.isdir(folder):
        update_header_guards(folder)
    else:
        print("Invalid folder path.")
