import markdown
from typing import List, Dict, Any
import os
from pathlib import Path


class MarkdownParser:
    """
    Utility class for parsing Markdown files and extracting content
    """

    @staticmethod
    def extract_text_from_md(file_path: str) -> str:
        """
        Extract plain text content from a Markdown file
        """
        with open(file_path, 'r', encoding='utf-8') as file:
            md_content = file.read()

        # Convert Markdown to HTML, then strip HTML tags to get plain text
        html = markdown.markdown(md_content)

        # Remove HTML tags to get plain text
        plain_text = MarkdownParser._strip_html_tags(html)
        return plain_text

    @staticmethod
    def _strip_html_tags(html: str) -> str:
        """
        Remove HTML tags from a string
        """
        import re
        clean = re.compile('<.*?>')
        return re.sub(clean, '', html)

    @staticmethod
    def extract_text_from_directory(directory_path: str, file_extensions: List[str] = None) -> Dict[str, str]:
        """
        Extract text content from all Markdown files in a directory
        Returns a dictionary mapping file paths to their text content
        """
        if file_extensions is None:
            file_extensions = ['.md', '.markdown']

        content_map = {}
        directory = Path(directory_path)

        for file_path in directory.rglob('*'):
            if file_path.suffix.lower() in file_extensions:
                try:
                    text_content = MarkdownParser.extract_text_from_md(str(file_path))
                    content_map[str(file_path)] = text_content
                except Exception as e:
                    print(f"Error processing {file_path}: {e}")

        return content_map

    @staticmethod
    def get_all_markdown_files(directory_path: str, file_extensions: List[str] = None) -> List[str]:
        """
        Get a list of all Markdown files in a directory
        """
        if file_extensions is None:
            file_extensions = ['.md', '.markdown']

        markdown_files = []
        directory = Path(directory_path)

        for file_path in directory.rglob('*'):
            if file_path.suffix.lower() in file_extensions:
                markdown_files.append(str(file_path))

        return markdown_files

    @staticmethod
    def parse_markdown_with_structure(file_path: str) -> Dict[str, Any]:
        """
        Parse Markdown file and return structured data including headings and content
        """
        with open(file_path, 'r', encoding='utf-8') as file:
            lines = file.readlines()

        structure = {
            'title': '',
            'headings': [],
            'content': '',
            'sections': []
        }

        current_section = {'heading': '', 'content': ''}
        content_lines = []

        for line in lines:
            # Check if line is a heading (starts with #)
            if line.strip().startswith('#'):
                # Save previous section if it exists
                if current_section['heading'] or current_section['content']:
                    structure['sections'].append({
                        'heading': current_section['heading'],
                        'content': current_section['content'].strip()
                    })

                # Extract heading text (remove # symbols)
                heading_text = line.strip().lstrip('#').strip()
                if not structure['title']:  # First heading is usually the title
                    structure['title'] = heading_text

                current_section = {
                    'heading': heading_text,
                    'content': ''
                }
                structure['headings'].append(heading_text)
            else:
                # Add line to current section content
                if current_section['heading']:
                    current_section['content'] += line
                else:
                    # Content before first heading goes to general content
                    content_lines.append(line)

        # Add the last section
        if current_section['heading'] or current_section['content']:
            structure['sections'].append({
                'heading': current_section['heading'],
                'content': current_section['content'].strip()
            })

        # Add content before first heading
        structure['content'] = ''.join(content_lines).strip()

        return structure