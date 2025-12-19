"""
Simple script to process book markdown files from docs directory
This script reads markdown files and prepares them for querying
"""
import os
import glob

def process_book_files():
    """Process all markdown files in the docs directory"""
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

if __name__ == "__main__":
    process_book_files()