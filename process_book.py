"""
Script to process book markdown files from docs directory
This script can either do simple processing or embed to Qdrant
"""
import os
import glob
import sys

def process_book_files_simple():
    """Process all markdown files in the docs directory (simple version)"""
    docs_path = "./docs"

    if not os.path.exists(docs_path):
        print(f"Docs directory does not exist: {docs_path}")
        return False

    markdown_files = glob.glob(f"{docs_path}/**/*.md", recursive=True)

    print(f"Found {len(markdown_files)} markdown files to process")

    total_chars = 0
    for file_path in markdown_files:
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
                total_chars += len(content)
                print(f"Processed: {file_path} ({len(content)} characters)")
        except Exception as e:
            print(f"Error processing {file_path}: {e}")

    print(f"Total characters processed: {total_chars}")
    print("Book processing completed!")

    return True

def embed_to_qdrant():
    """Embed book files to Qdrant (calls the embed script)"""
    import subprocess
    result = subprocess.run([sys.executable, "embed_to_qdrant.py"], capture_output=True, text=True)

    if result.returncode == 0:
        print("Successfully embedded book content to Qdrant")
        print(result.stdout)
    else:
        print("Error embedding to Qdrant:")
        print(result.stderr)

    return result.returncode == 0

def main():
    """Main function - can do simple processing or embedding"""
    if len(sys.argv) > 1 and sys.argv[1] == "embed":
        print("Embedding book content to Qdrant...")
        return embed_to_qdrant()
    else:
        print("Running simple book processing...")
        return process_book_files_simple()

if __name__ == "__main__":
    main()