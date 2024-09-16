#!/usr/bin/env python3
import elsie
from elsie.ext import unordered_list
from elsie.boxtree.box import Box

slides = elsie.SlideDeck(width=1920, height=1080)

# Update styles
slides.update_style("default", elsie.TextStyle(font="Lato", align="left", size=64))
slides.update_style("code", elsie.TextStyle(size=38))
slides.set_style("link", elsie.TextStyle(color="blue"))

# Helper functions for common slide elements
def logo_header_slide(parent: Box, title: str):
    # Add logo if needed
    # parent.box(x=1570, y=40).image("picknik_logo.png")
    parent.sbox(name="header", x=0, height=140).fbox(p_left=20).text(
        title, elsie.TextStyle(bold=True)
    )
    return parent.fbox(name="content", p_left=20, p_right=20)

def image_slide(parent: Box, title: str, image_path: str):
    content = logo_header_slide(parent, title)
    content = content.fbox(horizontal=True, p_top=20, p_bottom=20)
    text_area = content.fbox(name="text_area", width="50%")
    content.sbox(name="image", width="fill").image(image_path)
    return text_area

def code_slide(parent: Box, title: str, language: str, code: str):
    content = logo_header_slide(parent, title)
    code_bg = "#F6F8FA"
    box = content.box(y=0, width="100%", height="100%", p_bottom=20)
    box.rect(bg_color=code_bg, rx=20, ry=20)
    box.overlay().box(
        x=0, y=0, p_left=20, p_right=20, p_top=20, p_bottom=20
    ).code(language, code)

def add_footer(slide):
    slide.set_style("footer", elsie.TextStyle(color="black", size=32, align="right"))
    footer = slide.box(width="fill", height="5%", horizontal=True)
    f1 = footer.box(width="fill", height="fill")
    f1.text("github.com/griswaldbrooks/spanny", "footer")

# Slide 1: Title Slide
@slides.slide(debug_boxes=False)
def title_slide(slide):
    content = logo_header_slide(slide, "")
    content.box(width="fill").text(
        "spanny", elsie.TextStyle(size=160, bold=True)
    )
    content.box(width="fill").text(
        "Abusing mdspan is within arm's reach",
        elsie.TextStyle(size=72, bold=True, italic=True),
    )
    content.box(width="fill", p_top=10).text("October 5, 2023")
    content.box(width="fill", p_top=180).text("Griswald Brooks")
    content.box(width="fill").text("Senior Robotics Engineer\ngriswald.brooks@picknik.ai")

# Slide 2: Introduction
@slides.slide(debug_boxes=False)
def introduction(slide):
    content = logo_header_slide(slide, "Introduction")
    content.box().text(
        "What am I trying to convey?",
        elsie.TextStyle(size=80, bold=True),
    )
    ul = content.box().box()
    unordered_list(
        ul,
        [
            "How to write custom layouts and accessors in std::mdspan",
            "Applications in robotics",
            "Writing stateful layouts and async accessors",
            "Leveraging the full capabilities of C++",
        ],
        font_size=56,
    )
    add_footer(slide)

# Slide 3: Motivations for std::mdspan
@slides.slide(debug_boxes=False)
def motivations(slide):
    content = logo_header_slide(slide, "Motivations for std::mdspan")
    ul = content.box().box()
    unordered_list(
        ul,
        [
            "Limitations of previous non-owning types (std::span, std::string_view)",
            "Need for more flexible view types in C++",
            "References to talks by Bryce and Timur",
            "Reference Daisy Hollman's work",
        ],
        font_size=56,
    )
    add_footer(slide)

# Slide 4: Features of std::mdspan
@slides.slide(debug_boxes=False)
def features(slide):
    content = logo_header_slide(slide, "Features of std::mdspan")
    ul = content.box().box()
    unordered_list(
        ul,
        [
            "Multi-dimensional view type without memory layout restrictions",
            "Customization points: layout and accessor policies (strategy pattern)",
            "Layouts map multidimensional indices to storage locations",
            "Accessors retrieve data from storage locations",
            "Leveraging full flexibility of C++",
        ],
        font_size=56,
    )
    add_footer(slide)

# Slide 5: Layout Policies
@slides.slide(debug_boxes=False)
def layout_policies(slide):
    content = logo_header_slide(slide, "Layout Policies")
    ul = content.box().box()
    unordered_list(
        ul,
        [
            "What is a layout?",
            "Maps multidimensional index to a storage location",
            "Computers store data linearly; layouts provide mapping",
            "Layouts can be non-contiguous, have state, perform arbitrary computation",
            "Rank: number of extents; Extent: length of a dimension",
            "Out-of-the-box layouts: layout_right, layout_left, layout_stride",
            "Layout requirements and type definitions",
        ],
        font_size=56,
    )
    add_footer(slide)

# Slide 6: Occupancy Grids
@slides.slide(debug_boxes=False)
def occupancy_grids(slide):
    content = logo_header_slide(slide, "Occupancy Grids")
    content.box().text(
        "Implementing global occupancy grid with std::vector",
        elsie.TextStyle(size=56),
    )
    content.box().text(
        "A conventional use case in robotics for mapping environments.",
        elsie.TextStyle(size=56),
    )
    # Optionally add images or code snippets
    add_footer(slide)

# Slide 7: Submaps for Navigation
@slides.slide(debug_boxes=False)
def submaps(slide):
    content = logo_header_slide(slide, "Submaps for Navigation")
    ul = content.box().box()
    unordered_list(
        ul,
        [
            "Explanation of submaps/subgrids",
            "Challenges with layout_right for submaps",
            "Implementing submaps using layout_stride",
            "Using submdspan for easier implementation",
        ],
        font_size=56,
    )
    add_footer(slide)

# Slide 8: Rotated Submaps
@slides.slide(debug_boxes=False)
def rotated_submaps(slide):
    content = logo_header_slide(slide, "Rotated Submaps")
    ul = content.box().box()
    unordered_list(
        ul,
        [
            "Limitations of axis-aligned submaps",
            "Benefits of orientation-aware submaps",
            "Defining custom layouts for rotated views",
            "Exercise in index mapping with non-default constructors",
            "Using skew rotation algorithm mapping",
        ],
        font_size=56,
    )
    add_footer(slide)

# Slide 9: Polygonal Views and Arbitrary Layouts
@slides.slide(debug_boxes=False)
def polygonal_views(slide):
    content = logo_header_slide(slide, "Polygonal Views and Arbitrary Layouts")
    ul = content.box().box()
    unordered_list(
        ul,
        [
            "Extending layouts to non-square shapes",
            "Maintaining state within layouts",
            "Examples: layout_polygonal, layout_raytrace, layout_vertices",
            "Applications in collision checking and mapping",
        ],
        font_size=56,
    )
    add_footer(slide)

# Slide 10: Conclusion of Layout Policies
@slides.slide(debug_boxes=False)
def layout_conclusion(slide):
    content = logo_header_slide(slide, "Conclusion of Layout Policies")
    ul = content.box().box()
    unordered_list(
        ul,
        [
            "Learned what a layout is and its requirements",
            "Explored built-in layouts",
            "Created custom layouts with state and non-default constructors",
            "Discussed dimensionality reduction",
            "Previewed the need for custom accessors",
        ],
        font_size=56,
    )
    add_footer(slide)

# Slide 11: Accessor Policies
@slides.slide(debug_boxes=False)
def accessor_policies(slide):
    content = logo_header_slide(slide, "Accessor Policies")
    ul = content.box().box()
    unordered_list(
        ul,
        [
            "What are accessor policies?",
            "Retrieve data from storage locations using offsets",
            "Default accessor: return p[offset]",
            "Type requirements: element_type, reference, data_handle_type",
            "Accessors can call arbitrary functions",
            "Examples with custom accessors (e.g., Hilbert matrix)",
        ],
        font_size=56,
    )
    add_footer(slide)

# Slide 12: Roboticists' Beer Problem
@slides.slide(debug_boxes=False)
def beer_problem(slide):
    content = logo_header_slide(slide, "Roboticists' Beer Problem")
    ul = content.box().box()
    unordered_list(
        ul,
        [
            "Perennial problem: Not enough beer",
            "Historical examples: Willow Garage, MIT CSAIL",
            "Envisioning a 2x3 array allocated on the table",
            "Using robots to check beer supply",
        ],
        font_size=56,
    )
    add_footer(slide)

# Slide 13: Synchronous Command of a Robot Arm
@slides.slide(debug_boxes=False)
def sync_robot_arm(slide):
    content = logo_header_slide(slide, "Synchronous Robot Arm Control")
    ul = content.box().box()
    unordered_list(
        ul,
        [
            "Implementing a custom accessor for robot control",
            "Challenges with blocking functions in accessors",
            "Anatomy of the accessor type declaration and constructor",
            "Element type vs. data_handle_type in accessors",
            "Commanding the robot arm via accessor",
        ],
        font_size=56,
    )
    add_footer(slide)

# Slide 14: Synchronous Dual Robot Arms
@slides.slide(debug_boxes=False)
def sync_dual_arms(slide):
    content = logo_header_slide(slide, "Synchronous Dual Robot Arms")
    ul = content.box().box()
    unordered_list(
        ul,
        [
            "Adding another robot arm for efficiency",
            "Demonstration of dual arms",
            "Inefficiencies due to blocking",
            "Need for asynchronous operations",
        ],
        font_size=56,
    )
    add_footer(slide)

# Slide 15: Asynchronous Command with Dual Arms
@slides.slide(debug_boxes=False)
def async_dual_arms(slide):
    content = logo_header_slide(slide, "Asynchronous Dual Arm Control")
    ul = content.box().box()
    unordered_list(
        ul,
        [
            "Accessor::reference can be anything (e.g., std::future)",
            "Using std::jthread and std::packaged_task with thread-safe queue",
            "Accessors returning futures for async operations",
            "Leveraging full flexibility of C++ in accessors",
        ],
        font_size=56,
    )
    add_footer(slide)

# Slide 16: Summary of Accessor Policies
@slides.slide(debug_boxes=False)
def accessor_conclusion(slide):
    content = logo_header_slide(slide, "Summary of Accessor Policies")
    ul = content.box().box()
    unordered_list(
        ul,
        [
            "Understanding what accessors do and default behavior",
            "Requirements and non-requirements of accessors",
            "Implementing custom accessors from simple to complex",
            "Accessors can return async resources",
        ],
        font_size=56,
    )
    add_footer(slide)

# Slide 17: Conclusion
@slides.slide(debug_boxes=False)
def conclusion(slide):
    content = logo_header_slide(slide, "Conclusion")
    ul = content.box().box()
    unordered_list(
        ul,
        [
            "Discussed what mdspan is and its motivations",
            "Customization with layout and accessor policies",
            "Built-in options: layout_right, layout_left, layout_stride, submdspan, default_accessor",
            "Implementing custom policies with state and non-default constructors",
            "Accessors and layouts leveraging full flexibility of C++",
        ],
        font_size=56,
    )
    add_footer(slide)

# Slide 18: Final Slide
@slides.slide(debug_boxes=False)
def final_slide(slide):
    content = logo_header_slide(slide, "")
    content.fbox().text(
        "Thank you!",
        elsie.TextStyle(align="middle", size=80, bold=True),
    )
    content.fbox().text(
        "Code and slides on GitHub:\ngithub.com/griswaldbrooks/spanny\n\nCAD files and parts list to be published.",
        elsie.TextStyle(align="middle", size=56),
    )
    content.sbox(p_bottom=20).text(
        "Slides generated using Elsie",
        elsie.TextStyle(align="right", size=32),
    )
    add_footer(slide)

# Render the slides to a PDF
slides.render("spanny_talk.pdf")
