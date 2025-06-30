# Copyright 2025 Shivam Chudasama
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import tkinter as tk
from tkinter import font as tkFont
from tkinter import filedialog, ttk, messagebox
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
import io
try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    print("Pillow library not found. Please install with: pip install Pillow")
    # Or on Debian/Ubuntu: sudo apt-get install python3-pil

# --- Configuration for Drawing ---
NODE_WIDTH = 180
NODE_HEIGHT = 60
H_SPACING = 50
V_SPACING = 80
TEXT_PADDING = 10

# --- Theme Definitions ---
THEMES = {
    "Dark": {
        "bg": "#212121",
        "text": "#FFFFFF",
        "line": "#FFFFFF",
        "node_outline": "#FFFFFF",
        "Control": "#FFC107", "Action": "#4CAF50", "Condition": "#2196F3",
        "SubTree": "#9C27B0", "Decorator": "#FF9800", "Unknown": "#9E9E9E"
    },
    "Light": {
        "bg": "#FAFAFA",
        "text": "#000000",
        "line": "#424242",
        "node_outline": "#212121",
        "Control": "#FBC02D", "Action": "#388E3C", "Condition": "#1976D2",
        "SubTree": "#7B1FA2", "Decorator": "#F57C00", "Unknown": "#BDBDBD"
    },
    "Forest": {
        "bg": "#37493e",
        "text": "#130000", 
        "line": "#a3b18a",
        "node_outline": "#dad7cd",
        "Control": "#e76f51", "Action": "#a3b18a", "Condition": "#2a9d8f",
        "SubTree": "#f4a261", "Decorator": "#e9c46a", "Unknown": "#dad7cd"
    }
}


class BehaviorTreeNode:
    """Internal representation of a node in the behavior tree."""
    def __init__(self, element, depth=0):
        self.tag = element.tag
        self.attributes = element.attrib
        self.children = [BehaviorTreeNode(child, depth + 1) for child in element]
        self.depth = depth
        
        self.x = 0
        self.y = 0
        self.width = 0
        self.modifier = 0

    def get_display_name(self):
        """Generates the name to be displayed on the node."""
        node_id = self.attributes.get('ID', self.tag)
        instance_name = self.attributes.get('name', '')
        if instance_name:
            return f"{node_id}\n({instance_name})"
        return node_id

    def get_type(self):
        """Determines node type based on its tag for coloring."""
        tag = self.tag.lower()
        if 'sequence' in tag or 'fallback' in tag or 'parallel' in tag or 'reactivefallback' in tag:
            return "Control"
        if 'action' in tag: return "Action"
        if 'condition' in tag: return "Condition"
        if 'subtree' in tag: return "SubTree"
        
        decorators = ['decorator', 'inverter', 'retry', 'timeout', 'ratocontroller', 'keeprunninguntilfailure']
        if any(d in tag for d in decorators):
            return "Decorator"

        if self.tag in THEMES["Dark"]: # Check against a theme's keys
             return self.tag
        return "Unknown"


class BTRenderer(tk.Canvas):
    """A Tkinter canvas dedicated to rendering the behavior tree."""
    def __init__(self, parent, theme):
        super().__init__(parent, bg=theme["bg"])
        self.tree = None
        self.theme = theme
        self.scale_factor = 1.0
        self.base_font_size = 10
        self.canvas_font = tkFont.Font(family="Helvetica", size=self.base_font_size, weight="bold")

    def set_theme(self, theme):
        """Updates the canvas colors to match the selected theme."""
        self.theme = theme
        self.config(bg=theme["bg"])
        self.itemconfig("node_text", fill=theme["text"])
        self.itemconfig("line", fill=theme["line"])
        self.itemconfig("node_outline", outline=theme["node_outline"])

        # Recolor the node bodies
        for item_id in self.find_withtag("node_body"):
            node_type = self.gettags(item_id)[2] # Now the third tag is the node type
            self.itemconfig(item_id, fill=theme.get(node_type, theme["Unknown"]))

    def _wrap_text(self, text, max_width, font):
        """Wraps text to fit within a given width for the canvas font."""
        lines = []
        for paragraph in text.split('\n'):
            words = paragraph.split()
            if not words: continue
            current_line = words[0]
            for word in words[1:]:
                if font.measure(current_line + " " + word) <= max_width:
                    current_line += " " + word
                else:
                    lines.append(current_line)
                    current_line = word
            lines.append(current_line)
        return "\n".join(lines)

    def draw_tree(self, bt_root_node):
        """Clears the canvas and draws the new tree."""
        self.scale_factor = 1.0
        self.canvas_font.config(size=self.base_font_size)
        self.delete("all")
        if not bt_root_node or not bt_root_node.children:
            self.create_text(
                self.winfo_width() / 2, self.winfo_height() / 2,
                text="Load an XML file to view the Behavior Tree",
                fill=self.theme["text"], font=("Helvetica", 16)
            )
            return

        self.tree = bt_root_node.children[0]
        
        self.first_layout_pass(self.tree)
        self.second_layout_pass(self.tree, 0)
        self.draw_node_recursive(self.tree)
        self.update_scroll_region()

    def update_scroll_region(self):
        """Updates the scrollable area of the canvas, ensuring pending updates are processed first."""
        self.update_idletasks() # FIX: Force update of geometry before calculating bbox
        bbox = self.bbox("all")
        if bbox:
            self.config(scrollregion=(bbox[0] - 50, bbox[1] - 50, bbox[2] + 50, bbox[3] + 50))

    def first_layout_pass(self, node):
        """Post-order traversal to calculate subtree widths."""
        for child in node.children: self.first_layout_pass(child)
        if not node.children:
            node.width = NODE_WIDTH
            return
        children_width = -H_SPACING
        for child in node.children: children_width += child.width + H_SPACING
        node.width = max(NODE_WIDTH, children_width)
        next_x_start = -children_width / 2
        for child in node.children:
            child.modifier = next_x_start + child.width / 2
            next_x_start += child.width + H_SPACING

    def second_layout_pass(self, node, mod_sum):
        """Pre-order traversal to set final coordinates."""
        node.x = mod_sum
        node.y = (node.depth * (NODE_HEIGHT + V_SPACING)) + V_SPACING
        for child in node.children:
            self.second_layout_pass(child, mod_sum + child.modifier)

    def draw_node_recursive(self, node):
        """Recursively draws a node and its connections on the canvas."""
        center_offset_x = self.winfo_width() / 2
        x0 = node.x - NODE_WIDTH / 2 + center_offset_x
        y0 = node.y
        x1 = x0 + NODE_WIDTH
        y1 = y0 + NODE_HEIGHT
        
        node_type = node.get_type()
        color = self.theme.get(node_type, self.theme["Unknown"])
        
        self.create_rectangle(
            x0, y0, x1, y1,
            fill=color, outline=self.theme["node_outline"], width=2, 
            tags=("node", "node_body", node_type)
        )
        
        raw_text = node.get_display_name()
        wrapped_text = self._wrap_text(raw_text, NODE_WIDTH - TEXT_PADDING * 2, self.canvas_font)
        
        self.create_text(
            x0 + NODE_WIDTH / 2, y0 + NODE_HEIGHT / 2,
            text=wrapped_text, fill=self.theme["text"], font=self.canvas_font,
            justify=tk.CENTER, tags=("node", "node_text")
        )

        for child in node.children:
            px0, py0 = node.x + center_offset_x, y1
            cx0, cy0 = child.x + center_offset_x, child.y
            mid_y = py0 + V_SPACING / 2
            line_color = self.theme["line"]
            self.create_line(px0, py0, px0, mid_y, fill=line_color, width=1.5, tags="line")
            self.create_line(px0, mid_y, cx0, mid_y, fill=line_color, width=1.5, tags="line")
            self.create_line(cx0, mid_y, cx0, cy0, fill=line_color, width=1.5, tags="line")
            self.draw_node_recursive(child)

    def get_tree_bounds(self):
        """Calculates the logical bounding box of the entire tree."""
        if not self.tree:
            return None
        
        min_x, max_x = float('inf'), float('-inf')
        min_y, max_y = float('inf'), float('-inf')

        q = [self.tree]
        processed_nodes = {id(self.tree)}
        
        while q:
            node = q.pop(0)
            x0 = node.x - NODE_WIDTH / 2
            x1 = node.x + NODE_WIDTH / 2
            y0 = node.y
            y1 = node.y + NODE_HEIGHT
            
            min_x = min(min_x, x0)
            max_x = max(max_x, x1)
            min_y = min(min_y, y0)
            max_y = max(max_y, y1)
            
            for child in node.children:
                if id(child) not in processed_nodes:
                    q.append(child)
                    processed_nodes.add(id(child))

        return min_x, min_y, max_x, max_y

    def _draw_node_for_export(self, node, draw, font, offset_x, offset_y, dpi_scale):
        """Recursively draws the tree onto a Pillow ImageDraw context."""
        s_node_w = NODE_WIDTH * dpi_scale
        s_node_h = NODE_HEIGHT * dpi_scale
        s_v_space = V_SPACING * dpi_scale
        s_txt_pad = TEXT_PADDING * dpi_scale

        x0 = (node.x - NODE_WIDTH / 2 + offset_x) * dpi_scale
        y0 = (node.y + offset_y) * dpi_scale
        x1, y1 = x0 + s_node_w, y0 + s_node_h
        node_center_x = (node.x + offset_x) * dpi_scale
        
        node_type = node.get_type()
        color = self.theme.get(node_type, self.theme["Unknown"])
        draw.rectangle([x0, y0, x1, y1], fill=color, outline=self.theme["node_outline"], width=int(2 * dpi_scale))

        def wrap_export_text(text, font, max_width):
            lines = []
            for p in text.split('\n'):
                words = p.split()
                if not words: continue
                current_line = words[0]
                for word in words[1:]:
                    if draw.textbbox((0,0), f"{current_line} {word}", font=font)[2] <= max_width:
                        current_line += f" {word}"
                    else:
                        lines.append(current_line)
                        current_line = word
                lines.append(current_line)
            return "\n".join(lines)

        raw_text = node.get_display_name()
        wrapped_text = wrap_export_text(raw_text, font, s_node_w - s_txt_pad * 2)
        
        text_bbox = draw.textbbox((0,0), wrapped_text, font=font, align="center")
        text_w, text_h = text_bbox[2] - text_bbox[0], text_bbox[3] - text_bbox[1]
        text_x = x0 + (s_node_w - text_w) / 2
        text_y = y0 + (s_node_h - text_h) / 2
        draw.text((text_x, text_y), wrapped_text, font=font, fill=self.theme["text"], align="center")

        for child in node.children:
            child_center_x, child_y0 = (child.x + offset_x) * dpi_scale, (child.y + offset_y) * dpi_scale
            mid_y, line_width = y1 + s_v_space / 2, int(2 * dpi_scale)
            line_color = self.theme["line"]
            draw.line([node_center_x, y1, node_center_x, mid_y], fill=line_color, width=line_width)
            draw.line([node_center_x, mid_y, child_center_x, mid_y], fill=line_color, width=line_width)
            draw.line([child_center_x, mid_y, child_center_x, child_y0], fill=line_color, width=line_width)
            self._draw_node_for_export(child, draw, font, offset_x, offset_y, dpi_scale)

    def _on_zoom(self, event):
        """Handles zooming with Ctrl+MouseWheel, centering on the mouse cursor."""
        factor = 1.1 if event.num == 4 or event.delta == 120 else 0.9
        new_scale = self.scale_factor * factor
        if not (0.2 < new_scale < 5.0): return
        
        x = self.canvasx(event.x)
        y = self.canvasy(event.y)

        self.scale("all", 0, 0, factor, factor)

        delta_x = x - (x * factor)
        delta_y = y - (y * factor)

        self.move("all", delta_x, delta_y)
        
        self.scale_factor = new_scale
        
        new_font_size = max(1, int(self.base_font_size * self.scale_factor))
        self.canvas_font.config(size=new_font_size)
        self.update_scroll_region()

class BTVisualizerApp(tk.Tk):
    """The main application window."""
    def __init__(self, ros2_node):
        super().__init__()
        self.ros2_node = ros2_node
        self.title("ROS 2 Behavior Tree Visualizer")
        self.geometry("1200x800")
        self.bt_root = None
        self.current_file_path = None
        self.theme_name = tk.StringVar(value="Dark")
        self.current_theme = THEMES[self.theme_name.get()]

        self._pan_start_x = 0
        self._pan_start_y = 0

        main_frame = ttk.Frame(self)
        main_frame.pack(fill=tk.BOTH, expand=True)
        main_frame.rowconfigure(1, weight=1)
        main_frame.columnconfigure(0, weight=1)

        top_bar = ttk.Frame(main_frame)
        top_bar.grid(row=0, column=0, columnspan=2, sticky="ew", pady=5, padx=5)

        # --- Left-aligned buttons ---
        load_button = ttk.Button(top_bar, text="Load XML", command=self.prompt_load_xml)
        load_button.pack(side=tk.LEFT, padx=(10,5), pady=10)

        refresh_button = ttk.Button(top_bar, text="Refresh", command=self.refresh_xml)
        refresh_button.pack(side=tk.LEFT, padx=5, pady=10)

        save_button = ttk.Button(top_bar, text="Save as Image", command=self.save_as_image)
        save_button.pack(side=tk.LEFT, padx=5, pady=10)
        
        theme_label = ttk.Label(top_bar, text="Theme:")
        theme_label.pack(side=tk.LEFT, padx=(20, 5), pady=10)

        theme_selector = ttk.Combobox(top_bar, textvariable=self.theme_name, values=list(THEMES.keys()), state="readonly", width=10)
        theme_selector.pack(side=tk.LEFT, pady=10)
        theme_selector.bind("<<ComboboxSelected>>", self.change_theme)
        
        # --- Right-aligned info button ---
        info_button = ttk.Button(top_bar, text=" i ", command=self.show_info_dialog, width=3)
        info_button.pack(side=tk.RIGHT, padx=10, pady=10)

        # --- Center-aligned info label ---
        self.info_label = ttk.Label(top_bar, text="No file loaded.")
        self.info_label.pack(side=tk.LEFT, padx=20, fill=tk.X, expand=True)
        
        canvas_frame = ttk.Frame(main_frame)
        canvas_frame.grid(row=1, column=0, sticky="nsew")
        canvas_frame.rowconfigure(0, weight=1)
        canvas_frame.columnconfigure(0, weight=1)

        self.canvas = BTRenderer(canvas_frame, self.current_theme)
        self.canvas.grid(row=0, column=0, sticky="nsew")

        v_scroll = ttk.Scrollbar(canvas_frame, orient=tk.VERTICAL, command=self.canvas.yview)
        v_scroll.grid(row=0, column=1, sticky="ns")
        h_scroll = ttk.Scrollbar(canvas_frame, orient=tk.HORIZONTAL, command=self.canvas.xview)
        h_scroll.grid(row=1, column=0, sticky="ew")

        self.canvas.configure(yscrollcommand=v_scroll.set, xscrollcommand=h_scroll.set)
        
        self.canvas.bind("<ButtonPress-1>", self.on_pan_start)
        self.canvas.bind("<B1-Motion>", self.on_pan_move)
        self.canvas.bind("<ButtonPress-2>", self.on_pan_start)
        self.canvas.bind("<B2-Motion>", self.on_pan_move)
        self.canvas.bind("<Control-MouseWheel>", self.canvas._on_zoom)
        self.canvas.bind("<Command-MouseWheel>", self.canvas._on_zoom)
        self.canvas.bind("<Control-Button-4>", self.canvas._on_zoom)
        self.canvas.bind("<Control-Button-5>", self.canvas._on_zoom)
        self.canvas.bind("<Configure>", lambda e: self.canvas.draw_tree(self.bt_root))

    def show_info_dialog(self):
        """Displays a new window with information about the tool."""
        info_win = tk.Toplevel(self)
        info_win.title("Information")
        info_win.geometry("480x420")
        info_win.resizable(False, False)

        main_frame = ttk.Frame(info_win, padding="15")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # --- Summary ---
        summary_text = "A graphical tool to visualize ROS 2 Behavior Trees from XML files."
        summary_label = ttk.Label(main_frame, text=summary_text, wraplength=450, justify=tk.CENTER, font=("Helvetica", 11, "italic"))
        summary_label.pack(pady=(0, 15), anchor="w")

        # --- Controls ---
        controls_frame = ttk.LabelFrame(main_frame, text="Controls", padding="10")
        controls_frame.pack(fill=tk.X, pady=5)
        
        controls_text = (
            "Pan Canvas:      Hold & Drag Left or Middle Mouse Button\n"
            "Zoom Canvas:  Ctrl + Mouse Scroll Wheel (centers on cursor)"
        )
        controls_label = ttk.Label(controls_frame, text=controls_text, justify=tk.LEFT)
        controls_label.pack(anchor="w")

        # --- Color Schema ---
        schema_frame = ttk.LabelFrame(main_frame, text="Color Schema", padding="10")
        schema_frame.pack(fill=tk.X, pady=5)
        
        def create_schema_entry(parent, color, name, description):
            entry_frame = ttk.Frame(parent)
            color_box = tk.Frame(entry_frame, bg=color, width=20, height=20, relief="solid", borderwidth=1)
            color_box.pack(side=tk.LEFT, padx=(0, 10), pady=2)
            label_text = f"{name:<12} {description}"
            label = ttk.Label(entry_frame, text=label_text, justify=tk.LEFT)
            label.pack(side=tk.LEFT, anchor="w", fill=tk.X)
            entry_frame.pack(fill=tk.X, pady=2)

        theme = self.current_theme
        create_schema_entry(schema_frame, theme["Control"], "Control", "Directs logic flow (e.g., Sequence, Fallback).")
        create_schema_entry(schema_frame, theme["Action"], "Action", "Performs a task (e.g., MoveTo, Wait).")
        create_schema_entry(schema_frame, theme["Condition"], "Condition", "Checks a state (e.g., IsBatteryLow).")
        create_schema_entry(schema_frame, theme["SubTree"], "SubTree", "Executes a nested Behavior Tree.")
        create_schema_entry(schema_frame, theme["Decorator"], "Decorator", "Modifies its child's result (e.g., Inverter).")
        create_schema_entry(schema_frame, theme["Unknown"], "Unknown", "A custom or unrecognized node type.")
        
        info_win.transient(self)
        info_win.grab_set()
        self.wait_window(info_win)

    def change_theme(self, event=None):
        """Applies the selected theme to the application."""
        self.current_theme = THEMES[self.theme_name.get()]
        self.canvas.set_theme(self.current_theme)
        self.canvas.draw_tree(self.bt_root)

    def on_pan_start(self, event): self.canvas.scan_mark(event.x, event.y)
    def on_pan_move(self, event): self.canvas.scan_dragto(event.x, event.y, gain=1)

    def save_as_image(self):
        """Saves the current tree as a high-resolution PNG."""
        if not self.canvas.tree:
            messagebox.showwarning("Warning", "No Behavior Tree is loaded.")
            return

        file_path = filedialog.asksaveasfilename(
            title="Save Image As",
            filetypes=(("PNG files", "*.png"),),
            defaultextension=".png"
        )
        if not file_path: return

        try:
            bounds = self.canvas.get_tree_bounds()
            if not bounds:
                messagebox.showerror("Error", "Cannot determine tree bounds.")
                return

            min_x, min_y, max_x, max_y = bounds
            padding = 50
            
            logical_width = (max_x - min_x) + (2 * padding)
            logical_height = (max_y - min_y) + (V_SPACING + 2 * padding)
            
            dpi, dpi_scale = 300, 300 / 96.0
            img_width, img_height = int(logical_width * dpi_scale), int(logical_height * dpi_scale)

            img = Image.new('RGB', (img_width, img_height), color=self.current_theme["bg"])
            draw = ImageDraw.Draw(img)

            try:
                font = ImageFont.truetype("DejaVuSans-Bold.ttf", size=int(12 * dpi_scale))
            except IOError:
                self.ros2_node.get_logger().warn("DejaVuSans-Bold.ttf not found, using default font.")
                font = ImageFont.load_default()

            offset_x, offset_y = -min_x + padding, -min_y + padding
            self.canvas._draw_node_for_export(self.canvas.tree, draw, font, offset_x, offset_y, dpi_scale)
            
            img.save(file_path, 'png', dpi=(dpi, dpi))
            self.info_label.config(text=f"Saved image to {file_path.split('/')[-1]}")
            self.ros2_node.get_logger().info(f"Successfully saved image to {file_path}")

        except Exception as e:
            messagebox.showerror("Error", f"Failed to save image: {e}")
            self.ros2_node.get_logger().error(f"Failed to save image: {e}")

    def prompt_load_xml(self):
        """Opens a file dialog to select and parse an XML file."""
        file_path = filedialog.askopenfilename(title="Select BehaviorTree XML", filetypes=(("XML files", "*.xml"),))
        if file_path:
            self.load_xml(file_path)
    
    def refresh_xml(self):
        """Reloads the current XML file."""
        if self.current_file_path:
            self.load_xml(self.current_file_path)
            self.info_label.config(text=f"Refreshed: {self.current_file_path.split('/')[-1]}")
        else:
            messagebox.showinfo("Info", "No file is currently open to refresh.")

    def load_xml(self, file_path):
        """Loads and renders an XML file from a given path."""
        try:
            root_element = ET.parse(file_path).getroot()
            if root_element.tag != 'root' or 'BTCPP_format' not in root_element.attrib:
                 self.info_label.config(text="Error: Not a valid BT.CPP XML.")
                 self.ros2_node.get_logger().error("Not a valid BT.CPP V4 XML file.")
                 self.bt_root = None
                 self.current_file_path = None
            else:
                self.bt_root = BehaviorTreeNode(root_element)
                self.current_file_path = file_path # Store the path on successful load
            
            self.canvas.draw_tree(self.bt_root)
            if self.bt_root:
                self.info_label.config(text=f"Loaded: {file_path.split('/')[-1]}")
                self.ros2_node.get_logger().info(f"Successfully loaded {file_path}")

        except Exception as e:
            messagebox.showerror("Error", f"An error occurred: {e}")
            self.ros2_node.get_logger().error(f"An unexpected error occurred: {e}")

def main(args=None):
    rclpy.init(args=args)
    visualizer_node = Node('bt_visualizer_node')
    visualizer_node.get_logger().info('Behavior Tree Visualizer node started.')
    app = BTVisualizerApp(visualizer_node)
    
    def update_ros_and_tk():
        if rclpy.ok() and app.winfo_exists():
            rclpy.spin_once(visualizer_node, timeout_sec=0.01)
            app.after(10, update_ros_and_tk)
        elif not app.winfo_exists():
            rclpy.try_shutdown()

    try:
        app.after(10, update_ros_and_tk)
        app.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        visualizer_node.get_logger().info('Shutting down Behavior Tree Visualizer.')
        if rclpy.ok():
            visualizer_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
