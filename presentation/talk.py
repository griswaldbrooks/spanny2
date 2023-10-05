#!/usr/bin/env python3
import elsie
from elsie.ext import unordered_list
from elsie.boxtree.box import Box

slides = elsie.SlideDeck(width=1920, height=1080)

slides.update_style("default", elsie.TextStyle(font="Lato", align="left", size=64))
slides.update_style("code", elsie.TextStyle(size=38))
slides.set_style("link", elsie.TextStyle(color="blue"))


def logo_header_slide(parent: Box, title: str):
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


def section_title_slide(parent: Box, title: str, subtitle: str):
    content = logo_header_slide(parent, "")
    content.sbox().text(title, elsie.TextStyle(align="right", size=240, bold=True))
    content.box().text(subtitle)


def code_slide(parent: Box, title: str, language: str, code: str):
    content = logo_header_slide(parent, title)
    code_bg = "#F6F8FA"
    box = content.box(y=0, width="100%", height="100%", p_bottom=20)
    box.rect(bg_color=code_bg, rx=20, ry=20)
    box.overlay().box(x=0, y=0, p_left=20, p_right=20, p_top=20, p_bottom=20).code(
        language, code
    )


@slides.slide(debug_boxes=False)
def title(slide):
    content = logo_header_slide(slide, "")
    content.box(width="fill").text(
        "spanny", elsie.TextStyle(size=160, bold=True)
    )
    content.box(width="fill").text(
        "abusing mdspan is within arms reach", elsie.TextStyle(size=72, bold=True, italic=True))
    content.box(width="fill", p_top=10).text("October 4, 2023")
    content.box(width="fill", p_top=180).text("Griswald Brooks")
    content.box(width="fill").text("Senior Software Engineer\ngriswald.brooks@picknik.ai")

@slides.slide(debug_boxes=False)
def normal_usage(slide):
    code_slide(
        slide,
        "Getting Started",
        "C++",
        """
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("minimal_param_node");
  auto my_string = node->declare_parameter("my_string", "world");
  auto my_number = node->declare_parameter("my_number", 23);

  rclcpp::spin(node);
  rclcpp::shutdown();
}
""",
    )

@slides.slide(debug_boxes=False)
def demo_time(slide):
    content = logo_header_slide(slide, "")
    content.box().text("30 lines of C++ boilderpate per parameter")
    content.box(show="2").text(
        "Before handling of dynamic parameters", elsie.TextStyle(color="red")
    )

@slides.slide(debug_boxes=False)
def thank_you(slide):
    content = logo_header_slide(slide, "")
    content.fbox().text(
        "Thank you!\n~link{github.com/griswaldbrooks/spanny}\n",
        elsie.TextStyle(align="middle"),
    )
    content.sbox(p_bottom=20).text(
        "Slides generated using Elsie\n",
        elsie.TextStyle(align="right", size=32),
    )


slides.render("spanny_talk.pdf")
