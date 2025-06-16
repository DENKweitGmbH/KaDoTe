import tkinter as tk
# Added Radiobutton, StringVar
from tkinter import (filedialog, messagebox, Listbox, Scrollbar, Frame, Canvas,
                     Label, Button, END, SINGLE, VERTICAL, HORIZONTAL, RIGHT, Y,
                     BOTH, LEFT, BOTTOM, X, W, NW, DISABLED, NORMAL, SUNKEN, OptionMenu,
                     Toplevel, Checkbutton, BooleanVar, Radiobutton, StringVar, ttk)
from PIL import Image, ImageTk, ImageDraw
import os
import numpy as np
try:
    import cv2 # Import OpenCV
except ImportError:
    messagebox.showerror("Error", "OpenCV library not found. Please install it using: pip install opencv-python")
    exit()
try:
    import matplotlib.cm as cm
except ImportError:
    messagebox.showerror("Error", "Matplotlib library not found. Please install it using: pip install matplotlib")
    exit()
try:
    import open3d as o3d
except ImportError:
    messagebox.showerror("Error", "Open3D library not found. Please install it using: pip install open3d")
    # Allow to continue without 3D view functionality if Open3D is missing
try:
    from scipy.spatial import KDTree
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    # KDTree will be None if scipy is not installed
    # The application will still run, but nearest neighbor search for 3D points in alignment will be less efficient or disabled.
from methods import *

# Define zoom steps
ZOOM_STEP = 1.2 # Factor to zoom in/out by
MIN_ZOOM = 0.1
MAX_ZOOM = 5.0

# Define colors for points
COLOR_INACTIVE = "blue"
COLOR_ACTIVE = "cyan"
COLOR_TEMP = "red"

# Define Warp Methods
WARP_METHOD_HOMOGRAPHY = "Homography"
WARP_METHOD_TPS = "Thin Plate Spline"

# Define Point Cloud Heatmap defaults
HEATMAP_WIDTH = 512
HEATMAP_HEIGHT = 512

class PointPairSelector:
    def __init__(self, root):
        self.root = root
        self.root.title("Point Pair Selector")
        self.root.geometry("1200x700") # Increased initial size

        # --- Data ---
        self.image_path_a = None
        self.image_path_b = None
        self.is_b_point_cloud = False # Flag to indicate if B is a point cloud
        self.image_a = None # Original PIL Image A
        self.image_b = None # Original PIL Image B
        self.point_cloud_data_b = None # Stores loaded numpy array for point cloud B
        self.display_image_a = None # Resized PIL Image for display A
        self.display_image_b = None # Resized PIL Image for display B
        self.heatmap_image_b = None # Stores the full resolution generated heatmap for B

        self.tk_image_a = None # PhotoImage for Canvas A
        self.tk_image_b = None # PhotoImage for Canvas B
        self.point_pairs = [] # List to store ((xA, yA), (xB, yB)) - ORIGINAL coordinates
        self.temp_point_a = None # Store the first click (ORIGINAL coordinates)
        self.current_selection_index = None # Index of the pair selected in the listbox

        # --- Warp State ---
        self.warp_method = StringVar(value=WARP_METHOD_HOMOGRAPHY) # Default warp method
        self.homography_matrix = None # Store the calculated homography matrix
        self.tps_transformer = None   # Store the calculated TPS transformer (A -> B for remap)
        self.tps_homography_approx = None # Store the homography approximating the TPS warp

        # --- Visualization State ---
        self.show_all_points = BooleanVar(value=True) # Toggle state for showing all points

        # --- Alignment Window Data ---
        self.alignment_window = None # Reference to the Toplevel window
        self.alignment_canvas = None # Canvas in the Toplevel window
        self.blended_pil_image = None # Original blended PIL image (full size)
        self.tk_blended_image_display = None # PhotoImage for alignment display (potentially zoomed)
        self.alignment_listbox = None # Listbox in the alignment window
        self.alignment_point_pairs = [] # Points selected in the alignment window
        self.alignment_window_active_pc_slice_xyz = None # X,Y,Z data for current PC slice in alignment view
        self.alignment_current_selection_index = None # Index of selected point in alignment listbox
        self.alignment_zoom_level = 1.0

        # --- Zoom State ---
        # Image A transformation states
        self.image_a_rotation = 0  # Degrees: 0, 90, 180, 270
        self.image_a_flip_h = False
        self.image_a_flip_v = False

        self.zoom_level_a = 1.0
        self.zoom_level_b = 1.0

        # --- Point Cloud B Specific Data & UI ---
        self.pc_b_x_min_val = tk.DoubleVar()
        self.pc_b_x_max_val = tk.DoubleVar()
        self.pc_b_x_min_slider = None
        self.pc_b_x_max_slider = None
        self.btn_update_heatmap_b = None
        self.pc_b_colormap_var = tk.StringVar(value="viridis") # Default colormap
        self.available_colormaps = ["viridis", "plasma", "inferno", "magma", "cividis",
                                    "Greys", "Blues", "Reds", "hot", "coolwarm", "jet"]
        self.pc_b_colormap_menu = None
        self.btn_view_3d_pc_b = None

        # --- GUI Elements ---
        # (Initialize all GUI elements used in create_gui to None)
        self.canvas_a = self.canvas_b = self.listbox = self.status_label = None
        self.btn_load_a = self.btn_load_b = None
        self.btn_zoom_in_a = self.btn_zoom_out_a = self.btn_zoom_reset_a = None
        self.btn_rotate_a = None
        self.btn_flip_h_a = None
        self.btn_flip_v_a = None
        self.btn_zoom_in_b = self.btn_zoom_out_b = self.btn_zoom_reset_b = None
        self.btn_clear_selection = self.btn_remove_pair = None
        self.btn_calculate_warp = None # Renamed
        self.btn_show_alignment = None # Renamed
        self.chk_show_all_points = None
        self.btn_save_points = None
        self.btn_load_points = None
        self.btn_save_homography = None
        self.btn_load_homography = None
        self.btn_save_cam_pos = None

        self.create_gui()

        # --- State ---
        self.set_status("Load Image A and Image B to begin.")
        self._update_zoom_button_states()
        self._update_warp_button_states() # Renamed

        # --- Point Cloud Heatmap Coordinate Data ---
        self.heatmap_yz_min_max_b = None # To store (y_min, y_max, z_min, z_max) of data used for heatmap
        self.last_filtered_points_for_heatmap_xyz = None # Store X,Y,Z of points used for last heatmap

    def create_gui(self):
        # --- Main Frame for Images and List ---
        main_frame = Frame(self.root)
        main_frame.pack(fill=BOTH, expand=True, padx=10, pady=10)

        # --- PanedWindow for Image A and Image B ---
        image_paned_window = ttk.PanedWindow(main_frame, orient=HORIZONTAL)

        # --- Image A Frame ---
        # frame_a_container will be added to image_paned_window instead of main_frame directly
        frame_a_container = Frame(image_paned_window, width=500) # Initial width hint
        # frame_a_container.pack(side=LEFT, fill=BOTH, expand=True, padx=5) # Old packing
        image_paned_window.add(frame_a_container, weight=1) # Add to paned window

        # ... (Controls Frame A unchanged) ...
        controls_frame_a = Frame(frame_a_container)
        controls_frame_a.pack(pady=2, fill=X)
        self.btn_load_a = Button(controls_frame_a, text="Load Image A", command=self.load_image_a)
        self.btn_load_a.pack(side=LEFT, padx=5)
        Label(controls_frame_a, text="Zoom:").pack(side=LEFT, padx=(5, 2)) # Reduced left padding
        self.btn_zoom_in_a = Button(controls_frame_a, text="+", width=2, command=lambda: self.zoom("a", ZOOM_STEP))
        self.btn_zoom_in_a.pack(side=LEFT)
        self.btn_zoom_out_a = Button(controls_frame_a, text="-", width=2, command=lambda: self.zoom("a", 1/ZOOM_STEP))
        self.btn_zoom_out_a.pack(side=LEFT)
        self.btn_zoom_reset_a = Button(controls_frame_a, text="100%", command=lambda: self.zoom("a", 1.0, reset=True))
        self.btn_zoom_reset_a.pack(side=LEFT, padx=2)

        # Image A Transformation Buttons
        self.btn_rotate_a = Button(controls_frame_a, text="Rot 90°", command=self.rotate_image_a)
        self.btn_rotate_a.pack(side=LEFT, padx=(10,2))
        self.btn_flip_h_a = Button(controls_frame_a, text="Flip V", command=self.flip_image_a_horizontal)
        self.btn_flip_h_a.pack(side=LEFT, padx=2)
        self.btn_flip_v_a = Button(controls_frame_a, text="Flip H", command=self.flip_image_a_vertical)
        self.btn_flip_v_a.pack(side=LEFT, padx=2)
        Label(controls_frame_a, text="(Click first)").pack(side=LEFT, padx=(10,0)) # Adjusted padding
        # ... (Canvas A Frame unchanged) ...
        canvas_frame_a = Frame(frame_a_container, bd=1, relief=SUNKEN)
        canvas_frame_a.pack(fill=BOTH, expand=True)
        v_scroll_a = Scrollbar(canvas_frame_a, orient=VERTICAL)
        h_scroll_a = Scrollbar(canvas_frame_a, orient=HORIZONTAL)
        v_scroll_a.pack(side=RIGHT, fill=Y)
        h_scroll_a.pack(side=BOTTOM, fill=X)
        self.canvas_a = Canvas(canvas_frame_a, bg="lightgray", cursor="crosshair",
                               xscrollcommand=h_scroll_a.set,
                               yscrollcommand=v_scroll_a.set)
        self.canvas_a.pack(side=LEFT, fill=BOTH, expand=True)
        h_scroll_a.config(command=self.canvas_a.xview)
        v_scroll_a.config(command=self.canvas_a.yview)
        self.canvas_a.bind("<Button-1>", self.on_canvas_a_click)
        self.canvas_a.bind("<ButtonPress-2>", lambda event: self.canvas_a.scan_mark(event.x, event.y))
        self.canvas_a.bind("<B2-Motion>", lambda event: self.canvas_a.scan_dragto(event.x, event.y, gain=1))


        # --- Image B Frame ---
        # frame_b_container will be added to image_paned_window
        frame_b_container = Frame(image_paned_window, width=500) # Initial width hint
        # frame_b_container.pack(side=LEFT, fill=BOTH, expand=True, padx=5) # Old packing
        image_paned_window.add(frame_b_container, weight=1) # Add to paned window

        image_paned_window.pack(side=LEFT, fill=BOTH, expand=True, padx=(0,5)) # Pack paned window

        # ... (Controls Frame B unchanged) ...
        controls_frame_b = Frame(frame_b_container)
        controls_frame_b.pack(pady=2, fill=X) # Top controls for Image B
        self.btn_load_b = Button(controls_frame_b, text="Load Image B", command=self.load_image_b)
        self.btn_load_b.pack(side=LEFT, padx=5)
        Label(controls_frame_b, text="Zoom:").pack(side=LEFT, padx=(5, 2)) # Reduced left padding
        self.btn_zoom_in_b = Button(controls_frame_b, text="+", width=2, command=lambda: self.zoom("b", ZOOM_STEP))
        self.btn_zoom_in_b.pack(side=LEFT)
        self.btn_zoom_out_b = Button(controls_frame_b, text="-", width=2, command=lambda: self.zoom("b", 1/ZOOM_STEP))
        self.btn_zoom_out_b.pack(side=LEFT)
        self.btn_zoom_reset_b = Button(controls_frame_b, text="100%", command=lambda: self.zoom("b", 1.0, reset=True))
        self.btn_zoom_reset_b.pack(side=LEFT, padx=2)
        Label(controls_frame_b, text="(Click second)").pack(side=LEFT, padx=(10,5)) # Adjusted padding

        # --- Moved Point Cloud B Controls (Colormap, View 3D, Use Original YZ) ---
        # These are now part of controls_frame_b
        Label(controls_frame_b, text="Colormap:").pack(side=LEFT, padx=(10,2))
        self.pc_b_colormap_menu = OptionMenu(controls_frame_b, self.pc_b_colormap_var,
                                             *self.available_colormaps,
                                             command=self._on_pc_b_colormap_change)
        self.pc_b_colormap_menu.pack(side=LEFT, padx=2)

        self.btn_view_3d_pc_b = Button(controls_frame_b, text="View 3D",
                                     command=self.view_3d_point_cloud_b, state=DISABLED)
        self.btn_view_3d_pc_b.pack(side=LEFT, padx=5)

        # --- Point Cloud B Controls ---
        pc_controls_frame_b = Frame(frame_b_container)
        pc_controls_frame_b.pack(pady=2, fill=X)

        Label(pc_controls_frame_b, text="X-Range (for YZ Projection):").pack(side=LEFT, padx=5)

        self.pc_b_x_min_slider = tk.Scale(pc_controls_frame_b, from_=0, to=1, resolution=0.01,
                                          orient=HORIZONTAL, variable=self.pc_b_x_min_val,
                                          command=self._on_pc_b_x_range_change, length=100, state=DISABLED)
        self.pc_b_x_min_slider.pack(side=LEFT, padx=2)
        Label(pc_controls_frame_b, textvariable=self.pc_b_x_min_val).pack(side=LEFT, padx=2) # Show value

        self.pc_b_x_max_slider = tk.Scale(pc_controls_frame_b, from_=0, to=1, resolution=0.01,
                                          orient=HORIZONTAL, variable=self.pc_b_x_max_val,
                                          command=self._on_pc_b_x_range_change, length=100, state=DISABLED)
        self.pc_b_x_max_slider.pack(side=LEFT, padx=2)
        Label(pc_controls_frame_b, textvariable=self.pc_b_x_max_val).pack(side=LEFT, padx=2) # Show value

        self.btn_update_heatmap_b = Button(pc_controls_frame_b, text="Update Heatmap",
                                           command=self._trigger_heatmap_generation_b, state=DISABLED)
        self.btn_update_heatmap_b.pack(side=LEFT, padx=5)


        # ... (Canvas B Frame unchanged) ...
        canvas_frame_b = Frame(frame_b_container, bd=1, relief=SUNKEN)
        canvas_frame_b.pack(fill=BOTH, expand=True)
        v_scroll_b = Scrollbar(canvas_frame_b, orient=VERTICAL)
        h_scroll_b = Scrollbar(canvas_frame_b, orient=HORIZONTAL)
        v_scroll_b.pack(side=RIGHT, fill=Y)
        h_scroll_b.pack(side=BOTTOM, fill=X)
        self.canvas_b = Canvas(canvas_frame_b, bg="lightgray", cursor="crosshair",
                               xscrollcommand=h_scroll_b.set,
                               yscrollcommand=v_scroll_b.set)
        self.canvas_b.pack(side=LEFT, fill=BOTH, expand=True)
        h_scroll_b.config(command=self.canvas_b.xview)
        v_scroll_b.config(command=self.canvas_b.yview)
        self.canvas_b.bind("<Button-1>", self.on_canvas_b_click)
        self.canvas_b.bind("<ButtonPress-2>", lambda event: self.canvas_b.scan_mark(event.x, event.y))
        self.canvas_b.bind("<B2-Motion>", lambda event: self.canvas_b.scan_dragto(event.x, event.y, gain=1))


        # --- Right Panel (Listbox, Actions, Warp) ---
        right_panel = Frame(main_frame, width=250)
        right_panel.pack(side=RIGHT, fill=Y, padx=5)

        # Listbox Frame
        list_frame = Frame(right_panel, bd=1, relief=SUNKEN)
        list_frame.pack(fill=BOTH, expand=True, pady=(0, 5))
        # ... (Listbox setup unchanged) ...
        label_list = Label(list_frame, text="Point Pairs")
        label_list.pack()
        scrollbar = Scrollbar(list_frame, orient=VERTICAL)
        self.listbox = Listbox(list_frame, yscrollcommand=scrollbar.set, width=35, exportselection=False, selectmode=SINGLE)
        scrollbar.config(command=self.listbox.yview)
        scrollbar.pack(side=RIGHT, fill=Y)
        self.listbox.pack(side=LEFT, fill=BOTH, expand=True)
        self.listbox.bind("<<ListboxSelect>>", self.on_listbox_select)


        # Action Buttons Frame (Clear/Remove)
        action_buttons_frame = Frame(right_panel)
        action_buttons_frame.pack(fill=X, pady=5)
        # ... (Clear/Remove buttons unchanged) ...
        self.btn_clear_selection = Button(action_buttons_frame, text="Clear Current Selection", command=self.clear_current_selection, state=DISABLED)
        self.btn_clear_selection.pack(pady=2, fill=X)
        self.btn_remove_pair = Button(action_buttons_frame, text="Remove Selected Pair", command=self.remove_selected_pair, state=DISABLED)
        self.btn_remove_pair.pack(pady=2, fill=X)

        # Save/Load Points Buttons
        self.btn_save_points = Button(action_buttons_frame, text="Save Points",
                                      command=self.save_point_pairs, state=DISABLED)
        self.btn_save_points.pack(pady=(5,2), fill=X)
        self.btn_load_points = Button(action_buttons_frame, text="Load Points",
                                      command=self.load_point_pairs, state=NORMAL) # Initially enabled
        self.btn_load_points.pack(pady=2, fill=X)


        # Visualization Toggle
        vis_frame = Frame(right_panel)
        vis_frame.pack(fill=X, pady=5)
        # ... (Show All Points checkbox unchanged) ...
        self.chk_show_all_points = Checkbutton(vis_frame, text="Show All Points",
                                               variable=self.show_all_points,
                                               command=self._redraw_all_points)
        self.chk_show_all_points.pack(anchor=W)


        # --- Warp Control Frame ---
        warp_frame = Frame(right_panel, bd=1, relief=SUNKEN) # Add border
        warp_frame.pack(fill=X, pady=10, padx=5, ipady=5) # Add padding

        Label(warp_frame, text="Warp Method:").pack(anchor=W, padx=5)

        # Radio buttons for warp method selection
        warp_method_frame = Frame(warp_frame)
        warp_method_frame.pack(fill=X, padx=10)
        Radiobutton(warp_method_frame, text=WARP_METHOD_HOMOGRAPHY, variable=self.warp_method,
                    value=WARP_METHOD_HOMOGRAPHY, command=self._invalidate_warp).pack(anchor=W)
        Radiobutton(warp_method_frame, text=WARP_METHOD_TPS, variable=self.warp_method,
                    value=WARP_METHOD_TPS, command=self._invalidate_warp).pack(anchor=W)

        # Buttons for calculation and showing alignment
        self.btn_calculate_warp = Button(warp_frame, text="Calculate Warp",
                                         command=self.calculate_warp, state=DISABLED) # Renamed command
        self.btn_calculate_warp.pack(pady=(10, 2), fill=X, padx=5)

        self.btn_show_alignment = Button(warp_frame, text="Show Warped Alignment", # Renamed text
                                         command=self.show_alignment, state=DISABLED)
        self.btn_show_alignment.pack(pady=2, fill=X, padx=5)

        self.btn_save_homography = Button(warp_frame, text="Save Homography",
                                     command=self.save_homography_data, state=DISABLED)
        self.btn_save_homography.pack(pady=(5,2), fill=X, padx=5)
        self.btn_load_homography = Button(warp_frame, text="Load Homography",
                                        command=self.load_homography_data, state=NORMAL) # Initially enabled
        self.btn_load_homography.pack(pady=2, fill=X, padx=5)

        self.btn_save_cam_pos = Button(warp_frame, text="Save Camera Position",
                                     command=self.save_cam_pos_data, state=DISABLED)
        self.btn_save_cam_pos.pack(pady=(5,2), fill=X, padx=5)


        # --- Status Bar ---
        self.status_label = Label(self.root, text="", bd=1, relief=SUNKEN, anchor=W)
        self.status_label.pack(side=BOTTOM, fill=X, pady=(5,0))

    # --- Status and Button State Updates ---
    # ... (set_status, _update_zoom_button_states unchanged) ...
    def set_status(self, message):
        self.status_label.config(text=message)

    def _update_zoom_button_states(self):
        """Enable/disable zoom buttons based on whether images are loaded."""
        state_a = NORMAL if self.image_a else DISABLED
        state_b = NORMAL if self.image_b else DISABLED

        if self.btn_zoom_in_a: self.btn_zoom_in_a.config(state=state_a)
        if self.btn_zoom_out_a: self.btn_zoom_out_a.config(state=state_a)
        if self.btn_zoom_reset_a: self.btn_zoom_reset_a.config(state=state_a)
        if self.btn_rotate_a: self.btn_rotate_a.config(state=state_a)
        if self.btn_flip_h_a: self.btn_flip_h_a.config(state=state_a)
        if self.btn_flip_v_a: self.btn_flip_v_a.config(state=state_a)

        if self.btn_zoom_in_b: self.btn_zoom_in_b.config(state=state_b)
        if self.btn_zoom_out_b: self.btn_zoom_out_b.config(state=state_b)
        if self.btn_zoom_reset_b: self.btn_zoom_reset_b.config(state=state_b)

        # Point cloud controls for B
        pc_state = NORMAL if self.is_b_point_cloud and self.point_cloud_data_b is not None else DISABLED
        if self.pc_b_x_min_slider: self.pc_b_x_min_slider.config(state=pc_state)
        if self.pc_b_x_max_slider: self.pc_b_x_max_slider.config(state=pc_state)
        if self.btn_update_heatmap_b: self.btn_update_heatmap_b.config(state=pc_state)
        if self.pc_b_colormap_menu: self.pc_b_colormap_menu.config(state=pc_state)
        if self.btn_view_3d_pc_b: self.btn_view_3d_pc_b.config(state=pc_state)

    def _update_warp_button_states(self): # Renamed
        """Enable/disable warp buttons based on points and calculation status."""
        # Calculate button enabled only if >= 4 points and both images loaded
        # (Using 4 for both methods for simplicity, TPS technically needs 3+)
        can_calculate = len(self.point_pairs) >= 4 and self.image_a and self.image_b
        if self.btn_calculate_warp:
            self.btn_calculate_warp.config(state=NORMAL if can_calculate else DISABLED)

        # Show alignment button enabled only if a valid warp is calculated
        # If B is a point cloud, self.image_b will be the heatmap, which is a valid PIL Image.
        warp_calculated = (self.homography_matrix is not None) or (self.tps_transformer is not None)
        can_show = warp_calculated and self.image_a and self.image_b
        if self.btn_show_alignment:
            self.btn_show_alignment.config(state=NORMAL if can_show else DISABLED)

        if self.btn_save_homography:
            self.btn_save_homography.config(state=NORMAL if self.homography_matrix is not None else DISABLED)
        if self.btn_load_homography: # Always enabled, checks internally
            self.btn_load_homography.config(state=NORMAL)
        
        can_calculate_cam_pos = len(self.point_pairs) >= 6 and self.image_a and self.image_b
        if self.btn_save_cam_pos:
            self.btn_save_cam_pos.config(state=NORMAL if can_calculate_cam_pos else DISABLED)
        
        # Point list action buttons
        has_points = len(self.point_pairs) > 0
        if self.btn_save_points: self.btn_save_points.config(state=NORMAL if has_points else DISABLED)
        if self.btn_remove_pair: self.btn_remove_pair.config(state=NORMAL if self.current_selection_index is not None and has_points else DISABLED)


    def _invalidate_warp(self):
        """Resets calculated warp when method changes or points are modified."""
        self.homography_matrix = None
        self.tps_transformer = None
        self.tps_homography_approx = None # Also clear the (now unused) approx matrix
        self.blended_pil_image = None
        # self.heatmap_image_b = None # Don't clear heatmap here, only on new load B
        self._update_warp_button_states()
        if self.alignment_window and self.alignment_window.winfo_exists():
            self.alignment_window.destroy()
            self.alignment_window = None
        # Update status if method changed via radio button
        self.set_status(f"Warp method set to {self.warp_method.get()}. Recalculate warp.")


    # --- Image Loading and Display ---
    def load_image(self, target):
        """Loads an image for target 'a' or 'b'."""
        file_path = filedialog.askopenfilename(
            title=f"Select Image {target.upper()}",
            filetypes=[("Image Files", "*.png *.jpg *.jpeg *.bmp *.tif *.tiff"),
                       ("NumPy Point Cloud", "*.npy"), ("All Files", "*.*")]
        )
        if not file_path:
            self.set_status(f"Image {target.upper()} loading cancelled.")
            return False

        img_attr = f"image_{target}"
        path_attr = f"image_path_{target}"
        zoom_level_attr = f"zoom_level_{target}"

        file_extension = os.path.splitext(file_path)[1].lower()

        if target == 'b' and file_extension == '.npy':
            try:
                self.point_cloud_data_b = np.load(file_path)
                if self.point_cloud_data_b.ndim != 2 or self.point_cloud_data_b.shape[1] != 3:
                    raise ValueError("Point cloud data must be an N x 3 array.")

                self.is_b_point_cloud = True
                self.image_b = None # Clear any previous image for B
                self.heatmap_image_b = None # Clear previous heatmap
                self.display_image_b = None
                self.tk_image_b = None
                if self.canvas_b: self.canvas_b.delete("all")

                setattr(self, path_attr, file_path)
                setattr(self, zoom_level_attr, 1.0) # Reset zoom for heatmap

                # Determine X range from data
                if self.point_cloud_data_b.flags.f_contiguous: # Fortran order: [z,y,x]
                    x_coords = self.point_cloud_data_b[:, 2]
                else: # C-order: [x,y,z]
                    x_coords = self.point_cloud_data_b[:, 0]

                global_x_min = np.min(x_coords) if len(x_coords) > 0 else 0.0
                global_x_max = np.max(x_coords) if len(x_coords) > 0 else 1.0
                if global_x_min == global_x_max: # Handle case with single X value
                    global_x_max += 1.0

                self.pc_b_x_min_slider.config(from_=global_x_min, to=global_x_max)
                self.pc_b_x_max_slider.config(from_=global_x_min, to=global_x_max)
                self.pc_b_x_min_val.set(global_x_min)
                self.pc_b_x_max_val.set(global_x_max)

                self._invalidate_warp()
                self._trigger_heatmap_generation_b() # Generate and display initial heatmap

                self.set_status(f"Point cloud {os.path.basename(file_path)} loaded ({self.point_cloud_data_b.shape[0]} points). Adjust X-range for YZ projection.")
                self._update_zoom_button_states() # Updates all, including PC controls for B
                self._update_warp_button_states()
                return True

            except Exception as e:
                messagebox.showerror("Point Cloud Load Error", f"Failed to load point cloud {file_path}.\nError: {e}")
                self.set_status(f"Error loading Point Cloud B.")
                self.point_cloud_data_b = None
                self.is_b_point_cloud = False
                setattr(self, path_attr, None)
                self._invalidate_warp()
                self._update_zoom_button_states()
                self._update_warp_button_states()
                self._redraw_all_points()
                return False
        else: # Standard image loading for A or B (if B is not .npy)
            try:
                image = Image.open(file_path)
                if image.mode != 'RGB':
                    image = image.convert('RGB')

                setattr(self, img_attr, image)
                setattr(self, path_attr, file_path)
                setattr(self, zoom_level_attr, 1.0)
                self._invalidate_warp()

                if target == 'b':
                    self.is_b_point_cloud = False # Ensure flag is reset
                    self.point_cloud_data_b = None # Clear point cloud data
                    self.heatmap_image_b = None

                self._update_display_image(target)
                self.set_status(f"{os.path.basename(file_path)} loaded ({image.width}x{image.height})")
                self._update_zoom_button_states()
                self._update_warp_button_states()
                return True
            except Exception as e:
                messagebox.showerror("Image Load Error", f"Failed to load image {file_path}.\nError: {e}")
                self.set_status(f"Error loading Image {target.upper()}.")
                setattr(self, img_attr, None)
                setattr(self, path_attr, None)
                if target == 'b': self.is_b_point_cloud = False
                self._invalidate_warp()
                self._update_zoom_button_states()
                self._update_warp_button_states()
                self._redraw_all_points()
                return False

    def load_image_a(self):
        if self.load_image("a"):
            self.temp_point_a = None
            # Reset transformations for new image A
            self.image_a_rotation = 0
            self.image_a_flip_h = False
            self.image_a_flip_v = False
            self.set_status("Image A loaded. Click a point on Image A.")
            self.btn_clear_selection.config(state=DISABLED)

    def load_image_b(self):
        if self.load_image("b"):
            if self.is_b_point_cloud:
                # Status already set by point cloud loading logic
                pass
            elif self.temp_point_a:
                self.set_status("Image B loaded. Click the corresponding point on Image B.")
            elif self.image_a:
                self.set_status("Image B loaded. Click a point on Image A first.")
            else:
                self.set_status("Image B loaded.")
        self._update_zoom_button_states() # Ensure PC controls are updated

    # --- Image A Transformation Methods ---
    def _apply_transformations_to_pil_image_a(self, pil_image):
        """Applies current rotation and flips to a *copy* of PIL Image A."""
        if not pil_image:
            return None
        
        transformed_image = pil_image.copy()

        # Apply flips first, then rotation
        if self.image_a_flip_h:
            transformed_image = transformed_image.transpose(Image.FLIP_LEFT_RIGHT)
        if self.image_a_flip_v:
            transformed_image = transformed_image.transpose(Image.FLIP_TOP_BOTTOM)
        
        if self.image_a_rotation != 0:
            # PIL's rotate is counter-clockwise, so adjust if needed or use transpose
            if self.image_a_rotation == 90:
                transformed_image = transformed_image.transpose(Image.ROTATE_90)
            elif self.image_a_rotation == 180:
                transformed_image = transformed_image.transpose(Image.ROTATE_180)
            elif self.image_a_rotation == 270:
                transformed_image = transformed_image.transpose(Image.ROTATE_270)
        
        return transformed_image

    def rotate_image_a(self):
        if not self.image_a: return
        self.image_a_rotation = (self.image_a_rotation + 90) % 360
        self._update_display_image("a")
        self.set_status(f"Image A rotated to {self.image_a_rotation}°")

    def flip_image_a_horizontal(self):
        if not self.image_a: return
        self.image_a_flip_h = not self.image_a_flip_h
        self._update_display_image("a")
        self.set_status(f"Image A horizontal flip: {'On' if self.image_a_flip_h else 'Off'}")

    def flip_image_a_vertical(self):
        if not self.image_a: return
        self.image_a_flip_v = not self.image_a_flip_v
        self._update_display_image("a")
        self.set_status(f"Image A vertical flip: {'On' if self.image_a_flip_v else 'Off'}")


    def _get_transformed_image_a_for_display(self):
        """Returns the PIL Image A after applying stored transformations, without zoom."""
        if not self.image_a:
            return None
        
        # Start with the original image
        img_to_transform = self.image_a 
        # Apply visual transformations for display
        img_to_transform = self._apply_transformations_to_pil_image_a(img_to_transform)
        return img_to_transform

    def _update_display_image(self, target):
        """Updates the canvas image and scrollregion for target 'a' or 'b' based on zoom."""
        img_attr = f"image_{target}"
        tk_img_attr = f"tk_image_{target}"
        display_img_attr = f"display_image_{target}"
        zoom_level_attr = f"zoom_level_{target}"
        canvas = self.canvas_a if target == 'a' else self.canvas_b
        
        if target == 'a':
            # For Image A, get the version with visual transformations applied
            original_image_for_display = self._get_transformed_image_a_for_display()
        else: # For Image B (or heatmap)
            original_image_for_display = getattr(self, img_attr, None)
        zoom_level = getattr(self, zoom_level_attr, 1.0)

        if not original_image_for_display:
            # This case handles when image_b is None after loading a point cloud
            # before heatmap is generated, or if an image is cleared.
            if canvas:
                canvas.delete("all")
                canvas.config(scrollregion=(0, 0, 0, 0))
            setattr(self, tk_img_attr, None)
            setattr(self, display_img_attr, None)
            if target == 'b' and self.is_b_point_cloud and not self.heatmap_image_b:
                self.set_status("Generate heatmap for Point Cloud B.")
            # Ensure points are cleared if image is removed
            if target == 'a': self.image_a = None
            else: self.image_b = None
            self._redraw_all_points()
            return

        new_width_zoomed = int(original_image_for_display.width * zoom_level)
        new_height_zoomed = int(original_image_for_display.height * zoom_level)

        if new_width_zoomed < 1 or new_height_zoomed < 1:
            self.set_status(f"Zoom level too small for Image {target.upper()}.")
            if getattr(self, tk_img_attr, None):
                 if canvas:
                     canvas.delete("all")
                     canvas.config(scrollregion=(0, 0, 0, 0))
                 setattr(self, tk_img_attr, None)
                 setattr(self, display_img_attr, None)
                 self._redraw_all_points() # Clear points if display fails
            return

        resample_filter = Image.Resampling.LANCZOS

        try:
            if not canvas:
                print(f"Warning: Canvas for target '{target}' not initialized.")
                return

            display_image = original_image_for_display.resize((new_width_zoomed, new_height_zoomed), resample_filter)
            setattr(self, display_img_attr, display_image)

            tk_image = ImageTk.PhotoImage(display_image)
            setattr(self, tk_img_attr, tk_image)

            canvas.delete("all")
            canvas.create_image(0, 0, anchor=NW, image=tk_image, tags="image") # Displayed image is already transformed and zoomed
            canvas.config(scrollregion=(0, 0, new_width_zoomed, new_height_zoomed))

            self._redraw_all_points()

        except Exception as e:
            print(f"Error updating display image {target}: {e}")
            messagebox.showwarning("Zoom Error", f"Could not resize image {target.upper()} at this zoom level.\n{e}")
            if canvas:
                canvas.delete("all")
                canvas.config(scrollregion=(0, 0, 0, 0))
            setattr(self, tk_img_attr, None)
            setattr(self, display_img_attr, None)
            self._redraw_all_points()

    def zoom(self, target, factor, reset=False):
        """Zooms image 'a' or 'b' by a factor, or resets to 1.0."""
        zoom_level_attr = f"zoom_level_{target}"
        current_zoom = getattr(self, zoom_level_attr)

        if reset:
            new_zoom = 1.0
        else:
            new_zoom = current_zoom * factor
            new_zoom = max(MIN_ZOOM, min(MAX_ZOOM, new_zoom))

        if new_zoom != current_zoom:
            setattr(self, zoom_level_attr, new_zoom)
            self._update_display_image(target)
            self.set_status(f"Image {target.upper()} zoomed to {new_zoom:.1%}")
        elif reset:
             self.set_status(f"Image {target.upper()} zoom reset to 100%")
        else:
             limit = "maximum" if factor > 1 else "minimum"
             self.set_status(f"Reached {limit} zoom limit for Image {target.upper()}.")

    # --- Point Cloud B Specific Methods ---
    def _on_pc_b_x_range_change(self, value):
        """Callback when X-range sliders for Point Cloud B change."""
        min_val = self.pc_b_x_min_val.get()
        max_val = self.pc_b_x_max_val.get()

        if min_val > max_val:
            if self.pc_b_x_min_slider.get() == min_val : # min slider was moved
                self.pc_b_x_max_val.set(min_val)
            else: # max slider was moved
                self.pc_b_x_min_val.set(max_val)
        # No automatic heatmap update on slider move, user clicks button
        self.set_status(f"X-Range for YZ projection: [{self.pc_b_x_min_val.get():.2f}, {self.pc_b_x_max_val.get():.2f}]. Click 'Update Heatmap'.")

    def _on_pc_b_colormap_change(self, selected_colormap_name):
        """Callback when the colormap for Point Cloud B's heatmap is changed."""
        self.set_status(f"Colormap changed to '{selected_colormap_name}'. Updating heatmap...")
        self._trigger_heatmap_generation_b() # Regenerate heatmap with new colormap



    def _trigger_heatmap_generation_b(self):
        """Generates and displays the YZ heatmap for Point Cloud B."""
        if not self.point_cloud_data_b is not None or not self.is_b_point_cloud:
            self.set_status("No point cloud data loaded for B.")
            return

        self.set_status("Generating YZ heatmap for Point Cloud B...")
        self.root.update_idletasks()

        self.heatmap_image_b = self._generate_heatmap_pil_image_b()

        if self.heatmap_image_b:
            self.image_b = self.heatmap_image_b # This is now the "original" image for B
            self.display_image_b = None # Force regeneration of display image
            self.zoom_level_b = 1.0 # Reset zoom for new heatmap
            self._update_display_image("b")
            self._invalidate_warp() # Point data for B has effectively changed
            self.set_status(f"YZ heatmap generated and displayed. X-Range: [{self.pc_b_x_min_val.get():.2f}, {self.pc_b_x_max_val.get():.2f}]")
        else:
            if self.canvas_b: self.canvas_b.delete("all")
            self.image_b = None
            self.display_image_b = None
            self.tk_image_b = None
            self.set_status("Failed to generate heatmap or no data in selected X-range.")
        self._update_zoom_button_states()

    def _generate_heatmap_pil_image_b(self):
        """Filters point cloud B by X-range and generates a YZ projection heatmap PIL Image."""
        if self.point_cloud_data_b is None:
            return None

        x_min_filter = self.pc_b_x_min_val.get()
        x_max_filter = self.pc_b_x_max_val.get()

        # Determine indices based on memory layout
        is_fortran_order = self.point_cloud_data_b.flags.f_contiguous
        x_idx, y_idx, z_idx = (2, 1, 0) if is_fortran_order else (0, 1, 2)

        # Filter by X-range
        x_coords = self.point_cloud_data_b[:, x_idx]
        mask = (x_coords >= x_min_filter) & (x_coords <= x_max_filter)
        filtered_points = self.point_cloud_data_b[mask]
        
        if filtered_points.shape[0] == 0:
            self.last_filtered_points_for_heatmap_xyz = None # Clear it
            return None # No points in selected range

        # Ensure self.last_filtered_points_for_heatmap_xyz is in X,Y,Z order
        # This will be used by the alignment window for nearest neighbor search
        if is_fortran_order: # Original was [z,y,x]
            self.last_filtered_points_for_heatmap_xyz = filtered_points[:, [2, 1, 0]]
        else: # Original was [x,y,z]
            self.last_filtered_points_for_heatmap_xyz = filtered_points

        # Use y_coords_for_hist and z_coords_for_hist from the correctly ordered X,Y,Z slice for consistency
        # These are specifically for generating the 2D heatmap
        y_coords_for_hist = self.last_filtered_points_for_heatmap_xyz[:, 1] # Y column
        z_coords_for_hist = self.last_filtered_points_for_heatmap_xyz[:, 2] # Z column

        # Store min/max of Y and Z data used for this heatmap for coordinate conversion
        if y_coords_for_hist.size > 0 and z_coords_for_hist.size > 0:
            y_min_data, y_max_data = np.min(y_coords_for_hist), np.max(y_coords_for_hist)
            z_min_data, z_max_data = np.min(z_coords_for_hist), np.max(z_coords_for_hist)
            self.heatmap_yz_min_max_b = (y_min_data, y_max_data, z_min_data, z_max_data)
        else: # Should not happen if filtered_points.shape[0] > 0, but as a safeguard
            self.heatmap_yz_min_max_b = None
            return None

        # Create 2D histogram (heatmap)
        hist_range = [[y_min_data, y_max_data], [z_min_data, z_max_data]]
        heatmap, _, _ = np.histogram2d(y_coords_for_hist, z_coords_for_hist, bins=(HEATMAP_WIDTH, HEATMAP_HEIGHT), range=hist_range)
        heatmap = np.rot90(heatmap) # Rotate for typical image orientation (y-axis downwards)
        heatmap = np.flipud(heatmap) # Flip Y to match typical image coordinates (origin top-left)

        # Get selected colormap
        selected_colormap_name = self.pc_b_colormap_var.get()
        current_colormap = getattr(cm, selected_colormap_name, cm.viridis) # Fallback to viridis

        # Normalize and apply colormap
        heatmap_norm = (heatmap - np.min(heatmap)) / (np.max(heatmap) - np.min(heatmap) + 1e-9) # Avoid div by zero
        colored_heatmap = current_colormap(heatmap_norm) # RGBA

        # Convert to PIL Image
        pil_image = Image.fromarray((colored_heatmap[:, :, :3] * 255).astype(np.uint8), 'RGB')
        return pil_image

    def view_3d_point_cloud_b(self):
        """Displays the loaded point cloud for B in a 3D Open3D window."""
        if not self.is_b_point_cloud or self.point_cloud_data_b is None:
            messagebox.showinfo("3D View", "No point cloud loaded for Image B.")
            return
        
        try:
            o3d # Check if o3d was imported successfully
        except NameError:
            messagebox.showerror("3D View Error", "Open3D library is not available. Cannot display 3D point cloud.")
            return

        self.set_status("Preparing 3D view of point cloud B...")
        self.root.update_idletasks()

        try:
            pcd = o3d.geometry.PointCloud()
            
            # Get current X-filter range
            x_min_filter = self.pc_b_x_min_val.get()
            x_max_filter = self.pc_b_x_max_val.get()

            # Determine indices based on memory layout
            is_fortran_order = self.point_cloud_data_b.flags.f_contiguous
            x_idx_orig, y_idx_orig, z_idx_orig = (2, 1, 0) if is_fortran_order else (0, 1, 2)

            # Filter by X-range
            original_x_coords = self.point_cloud_data_b[:, x_idx_orig]
            mask = (original_x_coords >= x_min_filter) & (original_x_coords <= x_max_filter)
            filtered_pc_data = self.point_cloud_data_b[mask]

            if filtered_pc_data.shape[0] == 0:
                messagebox.showinfo("3D View", "No points in the selected X-range to display.")
                self.set_status("No points in selected X-range for 3D view.")
                return

            # Prepare points for Open3D (ensure X, Y, Z order)
            if is_fortran_order: # Original was [z,y,x]
                points_for_o3d = filtered_pc_data[:, [x_idx_orig, y_idx_orig, z_idx_orig]] # This is already [x,y,z] effectively after selection
            else: # Original was [x,y,z]
                points_for_o3d = filtered_pc_data

            pcd.points = o3d.utility.Vector3dVector(points_for_o3d[:, [0,1,2]]) # Ensure we take the first 3 columns if more exist

            # Color the point cloud by X-value (from the filtered data) using the selected colormap
            x_values_for_coloring = points_for_o3d[:, 0] # X is the first column in points_for_o3d

            if len(x_values_for_coloring) > 0:
                min_x_color, max_x_color = np.min(x_values_for_coloring), np.max(x_values_for_coloring)
                if max_x_color > min_x_color: # Avoid division by zero
                    normalized_x_color = (x_values_for_coloring - min_x_color) / (max_x_color - min_x_color)
                    selected_colormap_name = self.pc_b_colormap_var.get()
                    current_colormap = getattr(cm, selected_colormap_name, cm.viridis)
                    colors = current_colormap(normalized_x_color)[:, :3] # Get RGB, discard Alpha
                    pcd.colors = o3d.utility.Vector3dVector(colors)

            self.set_status("Displaying 3D point cloud. Close Open3D window to continue.")
            o3d.visualization.draw_geometries([pcd], window_name=f"Point Cloud B (X-Range: [{x_min_filter:.2f}, {x_max_filter:.2f}]) - 3D View")
            self.set_status("3D view closed.")
        except Exception as e:
            messagebox.showerror("3D View Error", f"Could not display 3D point cloud:\n{e}")
            self.set_status(f"Error displaying 3D point cloud: {e}")

    def _get_original_yz_from_heatmap_pixel(self, heatmap_pixel_x, heatmap_pixel_y):
        """Converts heatmap pixel coordinates to original YZ coordinates from the point cloud slice."""
        if self.heatmap_yz_min_max_b is None:
            return None

        y_min_data, y_max_data, z_min_data, z_max_data = self.heatmap_yz_min_max_b

        # Ensure HEATMAP_WIDTH and HEATMAP_HEIGHT are greater than 1 to avoid division by zero if they are 1
        # (though unlikely for a heatmap)
        denominator_w = (HEATMAP_WIDTH - 1.0) if HEATMAP_WIDTH > 1 else 1.0
        denominator_h = (HEATMAP_HEIGHT - 1.0) if HEATMAP_HEIGHT > 1 else 1.0

        # Pixel X (column) on heatmap corresponds to Y in point cloud
        # Pixel Y (row) on heatmap corresponds to Z in point cloud (after rot90 and flipud)
        original_y = y_min_data + (heatmap_pixel_x / denominator_w) * (y_max_data - y_min_data)
        
        # Due to np.flipud on the Z-axis representation:
        # heatmap pixel_y = 0 corresponds to z_max_data
        # heatmap pixel_y = HEATMAP_HEIGHT-1 corresponds to z_min_data
        original_z = z_max_data - (heatmap_pixel_y / denominator_h) * (z_max_data - z_min_data)
        
        return original_y, original_z

    def _get_heatmap_pixel_from_original_yz(self, original_y, original_z):
        """Converts original YZ coordinates from the point cloud slice back to heatmap pixel coordinates."""
        if self.heatmap_yz_min_max_b is None:
            return None

        y_min_data, y_max_data, z_min_data, z_max_data = self.heatmap_yz_min_max_b

        # Handle cases where the data range for an axis is zero (all points have the same Y or Z)
        if y_max_data == y_min_data:
            pixel_x = (HEATMAP_WIDTH -1) / 2.0 # Draw in the middle
        else:
            pixel_x = ((original_y - y_min_data) / (y_max_data - y_min_data)) * (HEATMAP_WIDTH - 1.0)

        # Account for np.flipud on Z: z_max_data maps to pixel_y = 0
        if z_max_data == z_min_data:
            pixel_y = (HEATMAP_HEIGHT -1) / 2.0 # Draw in the middle
        else:
            pixel_y = ((z_max_data - original_z) / (z_max_data - z_min_data)) * (HEATMAP_HEIGHT - 1.0)

        # Ensure pixel coordinates are within heatmap bounds, though they should be if original_y/z are from the data
        pixel_x = np.clip(pixel_x, 0, HEATMAP_WIDTH - 1)
        pixel_y = np.clip(pixel_y, 0, HEATMAP_HEIGHT - 1)

        return int(round(pixel_x)), int(round(pixel_y))

    # --- Coordinate Handling and Point Selection ---
    def _transform_coords_from_display_to_original_a(self, display_x, display_y):
        """
        Converts coordinates from the (unzoomed) transformed display space of Image A
        back to the original Image A's coordinate system.
        Assumes display_x, display_y are already unzoomed.
        """
        if not self.image_a: return None

        # display_x, display_y are coordinates on the unzoomed, transformed display.
        # We need to transform them back to the original image's coordinate system.

        # Step 1: Inverse Rotation
        # These coordinates will be in the space of the image *before* rotation, but *after* any flips were applied.
        x_after_inv_rotation = display_x
        y_after_inv_rotation = display_y

        # Inverse of rotation (PIL rotates CCW, so ROTATE_90 is CCW 90)
        # Let W_orig = self.image_a.width, H_orig = self.image_a.height
        if self.image_a_rotation == 90:
            # Display was (y_pre_flip, W_orig - 1 - x_pre_flip)
            # Inverse: x_pre_flip = W_orig - 1 - display_y; y_pre_flip = display_x
            x_after_inv_rotation = self.image_a.width - 1 - display_y
            y_after_inv_rotation = display_x
        elif self.image_a_rotation == 180:
            # Display was (W_orig - 1 - x_pre_flip, H_orig - 1 - y_pre_flip)
            # Inverse: x_pre_flip = W_orig - 1 - display_x; y_pre_flip = H_orig - 1 - display_y
            x_after_inv_rotation = self.image_a.width - 1 - display_x
            y_after_inv_rotation = self.image_a.height - 1 - display_y
        elif self.image_a_rotation == 270:
            # Display was (H_orig - 1 - y_pre_flip, x_pre_flip)
            # Inverse: x_pre_flip = display_y; y_pre_flip = H_orig - 1 - display_x
            x_after_inv_rotation = display_y
            y_after_inv_rotation = self.image_a.height - 1 - display_x

        # Step 2: Inverse Flips
        # Apply to x_after_inv_rotation, y_after_inv_rotation
        x_original = x_after_inv_rotation
        y_original = y_after_inv_rotation

        if self.image_a_flip_v:
            y_original = self.image_a.height - 1 - y_original
        if self.image_a_flip_h:
            x_original = self.image_a.width - 1 - x_original
            
        return int(round(x_original)), int(round(y_original))

    def _transform_coords_from_original_to_display_a(self, original_x, original_y):
        """
        Converts coordinates from the original Image A's space to the
        (unzoomed) transformed display space of Image A.
        """
        if not self.image_a: return None
        
        x, y = original_x, original_y
        
        # Apply flips first
        if self.image_a_flip_h:
            x = self.image_a.width - 1 - x
        if self.image_a_flip_v:
            y = self.image_a.height - 1 - y
            
        # Apply rotation (PIL rotates CCW)
        if self.image_a_rotation == 90: # Rotated 90 deg CCW
            x, y = y, self.image_a.width - 1 - x
        elif self.image_a_rotation == 180:
            x, y = self.image_a.width - 1 - x, self.image_a.height - 1 - y
        elif self.image_a_rotation == 270: # Rotated 270 deg CCW (or 90 CW)
            x, y = self.image_a.height - 1 - y, x
            
        return int(round(x)), int(round(y))

    def _get_original_coords(self, target, event):
        """Converts canvas click event coordinates to original image coordinates."""
        canvas = self.canvas_a if target == 'a' else self.canvas_b
        zoom_level = self.zoom_level_a if target == 'a' else self.zoom_level_b
        # For target 'a', display_image_a is already transformed and zoomed.
        # For target 'b', display_image_b is the (heatmap) zoomed.
        current_display_image = getattr(self, f"display_image_{target}", None)

        if not current_display_image: return None

        canvas_x = canvas.canvasx(event.x)
        canvas_y = canvas.canvasy(event.y)

        if 0 <= canvas_x < current_display_image.width and 0 <= canvas_y < current_display_image.height:
            # Coords on the unzoomed (but possibly transformed for A) image
            unzoomed_x = canvas_x / zoom_level
            unzoomed_y = canvas_y / zoom_level

            if target == 'a':
                # Convert from unzoomed-transformed-display coordinates to original image_a coordinates
                return self._transform_coords_from_display_to_original_a(unzoomed_x, unzoomed_y)
            else: # Target B (no visual transformations other than zoom)
                return int(unzoomed_x), int(unzoomed_y)
        else:
            return None

    def on_canvas_a_click(self, event):
        """Handles clicks on Image A canvas."""
        if not self.image_a:
            self.set_status("Load Image A first.")
            return

        coords = self._get_original_coords("a", event)

        if coords:
            original_x, original_y = coords # Unpack coordinates
            self.temp_point_a = (original_x, original_y)
            self.btn_clear_selection.config(state=NORMAL)

            status_msg = f"Point A selected at ({original_x},{original_y}). Now click the corresponding point on Image B."
            # Invalidate warp if already calculated
            if self.homography_matrix is not None or self.tps_transformer is not None:
                status_msg = f"Point A selected at ({original_x},{original_y}). Warp reset. Click point on Image B."
                self._invalidate_warp() # Reset warp state
            self.set_status(status_msg)
            self._redraw_all_points()
        else:
            self.set_status("Clicked outside Image A bounds.")


    # ... (on_canvas_b_click unchanged) ...
    def on_canvas_b_click(self, event):
        if not self.image_b:
            self.set_status("Load Image B first.")
            return
        if not self.temp_point_a:
            self.set_status("Click a point on Image A first.")
            return

        coords = self._get_original_coords("b", event)

        if coords: # Coords are pixel coordinates on the displayed image/heatmap
            point_b_to_store = None
            point_b_display_suffix = ""

            if self.is_b_point_cloud:
                if self.heatmap_yz_min_max_b is None or self.last_filtered_points_for_heatmap_xyz is None:
                    self.set_status("Heatmap data or filtered point cloud slice not ready for point selection.")
                    return
                if self.last_filtered_points_for_heatmap_xyz.shape[0] == 0:
                    self.set_status("No points in the current point cloud slice to select from.")
                    return

                # Convert heatmap click (coords are original heatmap pixels) to original YZ world coordinates
                original_yz_from_click = self._get_original_yz_from_heatmap_pixel(coords[0], coords[1])

                if original_yz_from_click:
                    world_y_click, world_z_click = original_yz_from_click
                    pc_slice_for_search = self.last_filtered_points_for_heatmap_xyz # N x 3 (X,Y,Z)

                    # Find the nearest 3D point in self.last_filtered_points_for_heatmap_xyz
                    if SCIPY_AVAILABLE:
                        kdtree = KDTree(pc_slice_for_search[:, 1:3]) # Query YZ columns
                        _, index = kdtree.query([world_y_click, world_z_click])
                        nearest_3d_point = pc_slice_for_search[index]
                    else: # Fallback to brute force
                        distances_sq = np.sum((pc_slice_for_search[:, 1:3] - np.array([world_y_click, world_z_click]))**2, axis=1)
                        index = np.argmin(distances_sq)
                        nearest_3d_point = pc_slice_for_search[index]
                    
                    point_b_to_store = tuple(nearest_3d_point) # Store (X,Y,Z)
                    point_b_display_suffix = f"PC({point_b_to_store[0]:.2f}, {point_b_to_store[1]:.2f}, {point_b_to_store[2]:.2f})"
                else:
                    self.set_status("Could not map heatmap click to world YZ coordinates.")
                    return
            else: # Image B is a regular image
                point_b_to_store = coords # Store (pixel_x, pixel_y)
                point_b_display_suffix = f"B{str(coords)}"

            if point_b_to_store:
                self.add_pair(self.temp_point_a, point_b_to_store)
                self.set_status(f"Pair added: A{self.temp_point_a} <-> {point_b_display_suffix}. Click next point on Image A.")
                self.temp_point_a = None
                self.btn_clear_selection.config(state=DISABLED)

        else:
            self.set_status("Clicked outside Image B bounds.")


    # --- Listbox and Pair Management ---
    def add_pair(self, point_a, point_b):
        # ... (Inside add_pair, call _invalidate_warp) ...
        self.point_pairs.append((point_a, point_b))
        self.update_listbox()
        self._invalidate_warp() # Reset warp state when points change
        self._redraw_all_points() # Redraw after adding

    # ... (update_listbox, on_listbox_select unchanged) ...
    def update_listbox(self):
        last_selected = self.listbox.curselection()
        self.listbox.delete(0, END)
        for i, (p_a, p_b) in enumerate(self.point_pairs):
            if self.is_b_point_cloud and isinstance(p_b, tuple) and len(p_b) == 3:
                # Point B is a 3D point from a point cloud (X, Y, Z)
                pb_str = f"PC({p_b[0]:.2f}, {p_b[1]:.2f}, {p_b[2]:.2f})"
            else: # Point B is 2D (either from a regular image or legacy point cloud data)
                pb_str = f"B({p_b[0]:.2f}, {p_b[1]:.2f})" if isinstance(p_b[0], float) or isinstance(p_b[1], float) else f"B{str(p_b)}"
            self.listbox.insert(END, f"{i+1}: A{p_a} <-> B{pb_str}")
        if last_selected:
            try:
                new_index = last_selected[0]
                if new_index < self.listbox.size():
                    self.listbox.selection_set(new_index)
                    self.listbox.activate(new_index)
                    self.listbox.see(new_index)
            except tk.TclError:
                pass

    def on_listbox_select(self, event):
        if not self.listbox.curselection():
            if self.current_selection_index is not None:
                 self.current_selection_index = None
                 self.btn_remove_pair.config(state=DISABLED)
                 self._redraw_all_points()
            return

        selected_indices = self.listbox.curselection()
        if selected_indices:
            new_selection_index = selected_indices[0]
            if new_selection_index != self.current_selection_index:
                self.current_selection_index = new_selection_index
                self.temp_point_a = None
                self.btn_clear_selection.config(state=DISABLED)
                self.set_status(f"Pair {self.current_selection_index + 1} selected.")
                self._redraw_all_points()

            self.btn_remove_pair.config(state=NORMAL)


    # --- Highlighting and Point Drawing ---
    # ... (_redraw_all_points, _draw_marker, highlight_pair, clear_highlights, clear_current_selection, scroll_to_view unchanged) ...
    def _redraw_all_points(self):
        """Clears old points and redraws all points based on current state."""
        if self.canvas_a:
            self.canvas_a.delete("point_marker")
        if self.canvas_b:
            self.canvas_b.delete("point_marker")

        if not self.show_all_points.get():
            return

        for i, (p_a, p_b) in enumerate(self.point_pairs):
            is_active = (i == self.current_selection_index)
            color = COLOR_ACTIVE if is_active else COLOR_INACTIVE
            tag = "point_marker"

            if self.image_a and self.canvas_a:
                # p_a is in original image coordinates. Transform to display space, then zoom.
                transformed_unzoomed_ax, transformed_unzoomed_ay = self._transform_coords_from_original_to_display_a(p_a[0], p_a[1])
                if transformed_unzoomed_ax is not None:
                    disp_ax_zoomed = transformed_unzoomed_ax * self.zoom_level_a
                    disp_ay_zoomed = transformed_unzoomed_ay * self.zoom_level_a
                    self._draw_marker(self.canvas_a, disp_ax_zoomed, disp_ay_zoomed, color, tag)


            if self.image_b and self.canvas_b:
                disp_bx_pixel_zoomed = None
                disp_by_pixel_zoomed = None

                if self.is_b_point_cloud and isinstance(p_b, tuple) and len(p_b) == 3:
                    # p_b is (X,Y,Z) from point cloud. Use Y (p_b[1]) and Z (p_b[2]) for heatmap pixel.
                    pixel_coords_b_on_heatmap = self._get_heatmap_pixel_from_original_yz(p_b[1], p_b[2])
                    if pixel_coords_b_on_heatmap:
                        disp_bx_pixel_zoomed = pixel_coords_b_on_heatmap[0] * self.zoom_level_b
                        disp_by_pixel_zoomed = pixel_coords_b_on_heatmap[1] * self.zoom_level_b
                elif isinstance(p_b, tuple) and len(p_b) == 2: # Regular image or legacy 2D point for PC
                    disp_bx_pixel_zoomed = p_b[0] * self.zoom_level_b
                    disp_by_pixel_zoomed = p_b[1] * self.zoom_level_b
                
                if disp_bx_pixel_zoomed is not None and disp_by_pixel_zoomed is not None:
                    self._draw_marker(self.canvas_b, disp_bx_pixel_zoomed, disp_by_pixel_zoomed, color, tag)


        if self.temp_point_a and self.image_a and self.canvas_a:
            # temp_point_a is in original image coordinates. Transform to display space, then zoom.
            transformed_unzoomed_ax, transformed_unzoomed_ay = self._transform_coords_from_original_to_display_a(self.temp_point_a[0], self.temp_point_a[1])
            if transformed_unzoomed_ax is not None:
                disp_ax_zoomed = transformed_unzoomed_ax * self.zoom_level_a
                disp_ay_zoomed = transformed_unzoomed_ay * self.zoom_level_a
                self._draw_marker(self.canvas_a, disp_ax_zoomed, disp_ay_zoomed, COLOR_TEMP, "point_marker")

    def _draw_marker(self, canvas, x, y, color, tag):
        """Draws a single point marker (circle + crosshair)."""
        if not canvas: return
        radius = 5
        cross_len = 7
        ix, iy = int(x), int(y)
        canvas.create_oval(ix - radius, iy - radius, ix + radius, iy + radius,
                           outline=color, width=2, tags=tag)
        canvas.create_line(ix - cross_len, iy, ix + cross_len, iy, fill=color, width=2, tags=tag)
        canvas.create_line(ix, iy - cross_len, ix, iy + cross_len, fill=color, width=2, tags=tag)

    def highlight_pair(self, index, redraw_only=False):
        """Sets the active index and redraws all points."""
        if index is None or index < 0 or index >= len(self.point_pairs):
             self.current_selection_index = None
        else:
             self.current_selection_index = index
        self._redraw_all_points()
        if self.current_selection_index is not None and not redraw_only:
            point_a_orig, point_b_orig = self.point_pairs[self.current_selection_index]
            if self.image_a and self.canvas_a:
                # point_a_orig is in original image coordinates. Transform to display space, then zoom for scroll target.
                transformed_unzoomed_ax, transformed_unzoomed_ay = self._transform_coords_from_original_to_display_a(point_a_orig[0], point_a_orig[1])
                if transformed_unzoomed_ax is not None:
                    disp_a_x_zoomed = transformed_unzoomed_ax * self.zoom_level_a
                    disp_a_y_zoomed = transformed_unzoomed_ay * self.zoom_level_a
                    self.scroll_to_view(self.canvas_a, disp_a_x_zoomed, disp_a_y_zoomed)

            if self.image_b and self.canvas_b:
                disp_b_x = point_b_orig[0] * self.zoom_level_b
                disp_b_y = point_b_orig[1] * self.zoom_level_b
                self.scroll_to_view(self.canvas_b, disp_b_x, disp_b_y)

    def clear_highlights(self, clear_temp=True):
        """Calls redraw which handles clearing and drawing based on state."""
        if clear_temp:
            self.temp_point_a = None
        self._redraw_all_points()

    def clear_current_selection(self):
        """Clears the temporary point A selection and redraws."""
        if self.temp_point_a:
            self.temp_point_a = None
            self.set_status("Selection cleared. Click a point on Image A.")
            self.btn_clear_selection.config(state=DISABLED)
            self._redraw_all_points()

    def scroll_to_view(self, canvas, x, y):
        """Scrolls the canvas so the point (x, y) is visible."""
        if not canvas or not canvas.winfo_exists(): return

        canvas_width = canvas.winfo_width()
        canvas_height = canvas.winfo_height()
        if canvas_width <= 1 or canvas_height <= 1: return

        scroll_region = canvas.cget("scrollregion")
        if not scroll_region or not scroll_region.strip(): return
        try:
            parts = list(map(float, scroll_region.split()))
            if len(parts) != 4: return
            _, _, content_width, content_height = parts
        except ValueError:
             return

        if content_width <= 0 or content_height <= 0: return

        target_x_fraction = (x - canvas_width / 2) / content_width
        target_y_fraction = (y - canvas_height / 2) / content_height

        visible_x_fraction = canvas_width / content_width
        visible_y_fraction = canvas_height / content_height

        max_x_frac = max(0.0, 1.0 - visible_x_fraction)
        max_y_frac = max(0.0, 1.0 - visible_y_fraction)
        target_x_fraction = max(0.0, min(target_x_fraction, max_x_frac))
        target_y_fraction = max(0.0, min(target_y_fraction, max_y_frac))

        canvas.xview_moveto(target_x_fraction)
        canvas.yview_moveto(target_y_fraction)


    def remove_selected_pair(self):
        # ... (Inside remove_selected_pair, call _invalidate_warp) ...
        if self.current_selection_index is None or not (0 <= self.current_selection_index < len(self.point_pairs)):
            # ... (return if no selection) ...
            return

        removed_pair = self.point_pairs.pop(self.current_selection_index)
        original_index = self.current_selection_index
        self.current_selection_index = None
        self.btn_remove_pair.config(state=DISABLED)
        self._invalidate_warp() # Reset warp state when points change

        self.update_listbox()
        self._update_warp_button_states() # Use renamed method

        # ... (rest of method, including selecting next/redrawing) ...
        new_selection_index = min(original_index, len(self.point_pairs) - 1)
        if new_selection_index >= 0:
             self.listbox.selection_set(new_selection_index)
             self.listbox.activate(new_selection_index)
             self.on_listbox_select(None)
             self.set_status(f"Removed pair: A{removed_pair[0]} <-> B{removed_pair[1]}. Selected pair {new_selection_index + 1}.")
        else:
             self._redraw_all_points()
             self.set_status(f"Removed pair: A{removed_pair[0]} <-> B{removed_pair[1]}. List is now empty.")


    # --- Warp Calculation ---

    def calculate_warp(self): # Renamed
        """Calculates the warp based on the selected method and point pairs."""
        if len(self.point_pairs) < 4: # Keep minimum 4 for robustness
            messagebox.showwarning("Warp Error", "Need at least 4 point pairs to calculate warp.")
            return
        if not self.image_a or not self.image_b:
             messagebox.showwarning("Warp Error", "Both Image A and Image B must be loaded.")
             return

        # Always reset previous results before calculating new one
        self.homography_matrix = None
        self.tps_transformer = None
        self.tps_homography_approx = None # Reset approx matrix
        self.blended_pil_image = None # Invalidate blend

        # Extract points into NumPy arrays (format needed by OpenCV)
        # Source points are from Image B, Target points are from Image A
        points_a_np_list = []
        points_b_np_list = []

        for pair_idx, (pa, pb) in enumerate(self.point_pairs):
            points_a_np_list.append(pa)
            if self.is_b_point_cloud and isinstance(pb, tuple) and len(pb) == 3:
                # Use Y (pb[1]) and Z (pb[2]) coordinates for warp from 3D point cloud data
                points_b_np_list.append((pb[1], pb[2]))
            elif isinstance(pb, tuple) and len(pb) == 2: # Standard 2D image or legacy 2D point for PC
                points_b_np_list.append(pb)
            else:
                messagebox.showerror("Warp Error", f"Invalid format for Point B in pair {pair_idx + 1}.")
                return

        points_a_np = np.float32(points_a_np_list)
        points_b_np = np.float32(points_b_np_list)

        method = self.warp_method.get()
        self.set_status(f"Calculating warp using {method}...")
        self.root.update_idletasks()

        try:
            if method == WARP_METHOD_HOMOGRAPHY:
                # Reshape for findHomography
                points_a_cv = points_a_np.reshape(-1, 1, 2)
                points_b_cv = points_b_np.reshape(-1, 1, 2)
                matrix, mask = cv2.findHomography(points_b_cv, points_a_cv, cv2.RANSAC, 5.0)
                if matrix is None:
                    raise ValueError("Homography calculation failed. Check point correspondences.")
                self.homography_matrix = matrix
                # Optional: Check if matrix is close to identity (might indicate poor points)
                identity = np.identity(3)
                diff = np.sum(np.abs(self.homography_matrix - identity))
                if diff < 0.1: # Arbitrary small threshold
                    print("Warning: Calculated homography matrix is very close to identity.")
                inlier_count = np.sum(mask) if mask is not None else len(points_a_np)
                self.set_status(f"Homography calculated successfully using {inlier_count}/{len(points_a_np)} inliers.")

            elif method == WARP_METHOD_TPS:
                # Use points directly (N, 2)
                tps = cv2.createThinPlateSplineShapeTransformer()

                # Reshape for estimateTransformation: (1, N, 2)
                matches = [cv2.DMatch(i, i, 0) for i in range(len(points_a_np))]
                # To warp Image B onto Image A's coordinate system using cv2.remap,
                # we need a transformation that maps points from A's space to B's space.
                # estimateTransformation(src, dst, ...) learns src -> dst.
                # So, src=points_a_np, dst=points_b_np to learn A -> B.
                tps.estimateTransformation(points_a_np.reshape(1, -1, 2), points_b_np.reshape(1, -1, 2), matches)
                self.tps_transformer = tps # Store the A -> B transformer
                self.set_status(f"TPS transformation calculated using {len(points_a_np)} points (for remap).")
                
        except Exception as e:
            import traceback # Import traceback module here or at the top
            traceback.print_exc() # Print traceback to console
            messagebox.showerror("Warp Error", f"An error occurred during {method} calculation:\n{e}")
            # Ensure state is reset on error
            self.homography_matrix = None
            self.tps_transformer = None
            self.tps_homography_approx = None
            self.set_status(f"{method} calculation failed.")

        self._update_warp_button_states() # Update button states (enables Show Alignment if successful)
        # Close alignment window if open, as warp changed
        if self.alignment_window and self.alignment_window.winfo_exists():
            self.alignment_window.destroy()
            self.alignment_window = None


    def save_point_pairs(self):
        if not self.point_pairs:
            messagebox.showinfo("Save Points", "No point pairs to save.")
            return

        filepath = filedialog.asksaveasfilename(
            title="Save Point Pairs",
            defaultextension=".txt",
            filetypes=[("Text Files", "*.txt"), ("CSV Files", "*.csv"), ("All Files", "*.*")]
        )
        if not filepath:
            self.set_status("Save point pairs cancelled.")
            return

        try:
            with open(filepath, 'w') as f:
                for pa, pb in self.point_pairs:
                    pb_str_parts = []
                    for coord in pb: # pb can be 2-tuple or 3-tuple
                        pb_str_parts.append(str(coord))
                    f.write(f"{pa[0]},{pa[1]}," + ",".join(pb_str_parts) + "\n")

            self.set_status(f"Point pairs saved to {os.path.basename(filepath)}")
        except Exception as e:
            messagebox.showerror("Save Error", f"Failed to save point pairs:\n{e}")
            self.set_status(f"Error saving point pairs: {e}")

    def load_point_pairs(self):
        filepath = filedialog.askopenfilename(
            title="Load Point Pairs",
            filetypes=[("Text Files", "*.txt"), ("CSV Files", "*.csv"), ("All Files", "*.*")]
        )
        if not filepath:
            self.set_status("Load point pairs cancelled.")
            return

        loaded_pairs = []
        try:
            with open(filepath, 'r') as f:
                for i, line in enumerate(f):
                    parts = line.strip().split(',')
                    if not (4 <= len(parts) <= 5): # Allow 4 (2D for B) or 5 (3D for B)
                        raise ValueError(f"Line {i+1}: Each line must contain 4 or 5 coordinate values.")
                    
                    pa = (int(parts[0]), int(parts[1]))
                    
                    pb_coords = []
                    for val_str in parts[2:]: # parts[2:] will be 2 or 3 items
                        try:
                            val_float = float(val_str)
                            pb_coords.append(int(val_float) if val_float.is_integer() else val_float)
                        except ValueError:
                             # If it's not a float, try int directly. If that fails, it's an error.
                            pb_coords.append(int(val_str))

                    pb = tuple(pb_coords)
                    loaded_pairs.append((pa, pb))

            self.point_pairs = loaded_pairs
            self.temp_point_a = None
            self.current_selection_index = None
            self.update_listbox()
            self._redraw_all_points()
            self._invalidate_warp() # Important to reset warp state
            self._update_warp_button_states()
            self.btn_clear_selection.config(state=DISABLED)
            self.set_status(f"{len(loaded_pairs)} point pairs loaded from {os.path.basename(filepath)}")
        except Exception as e:
            messagebox.showerror("Load Error", f"Failed to load point pairs:\n{e}")
            self.set_status(f"Error loading point pairs: {e}")


    def save_homography_data(self):
        if self.homography_matrix is None:
            messagebox.showinfo("Save Homography", "No homography matrix has been calculated yet.")
            return

        filepath = filedialog.asksaveasfilename(
            title="Save Homography Data",
            defaultextension=".npz",
            filetypes=[("NumPy Archive", "*.npz"), ("All Files", "*.*")]
        )
        if not filepath:
            self.set_status("Save homography cancelled.")
            return

        try:
            np.savez(filepath,
                     homography_matrix=self.homography_matrix,
                     warp_method=WARP_METHOD_HOMOGRAPHY, # Explicitly saving as homography
                     image_a_path=self.image_path_a if self.image_path_a else "N/A",
                     image_b_path=self.image_path_b if self.image_path_b else "N/A")
            self.set_status(f"Homography data saved to {os.path.basename(filepath)}")
        except Exception as e:
            messagebox.showerror("Save Error", f"Failed to save homography data:\n{e}")
            self.set_status(f"Error saving homography: {e}")

    def load_homography_data(self):
        filepath = filedialog.askopenfilename(
            title="Load Homography Data",
            filetypes=[("NumPy Archive", "*.npz"), ("All Files", "*.*")]
        )
        if not filepath:
            self.set_status("Load homography cancelled.")
            return
        try:
            data = np.load(filepath)
            if 'homography_matrix' not in data or data.get('warp_method') != WARP_METHOD_HOMOGRAPHY:
                messagebox.showerror("Load Error", "Invalid homography file or not a homography.")
                return

            self.homography_matrix = data['homography_matrix']
            self.warp_method.set(WARP_METHOD_HOMOGRAPHY)
            self.tps_transformer = None # Clear TPS if homography is loaded
            self.blended_pil_image = None # Force re-blend
            self._update_warp_button_states()
            self.set_status(f"Homography data loaded from {os.path.basename(filepath)}. Image A: {data.get('image_a_path', 'N/A')}, Image B: {data.get('image_b_path', 'N/A')}")
            if self.alignment_window and self.alignment_window.winfo_exists(): self.alignment_window.destroy()
        except Exception as e:
            messagebox.showerror("Load Error", f"Failed to load homography data:\n{e}")
            self.set_status(f"Error loading homography: {e}")

    def save_cam_pos_data(self):
        calibration_data_path = "camera_calibration.json"
        camera_position_data_output_path = "camera_position.json"

        image_points = []
        object_points = []

        for point_pair in self.point_pairs:
            image_points.append(point_pair[0])
            object_points.append(point_pair[1])

        image_points_np = np.array(image_points, dtype=np.float64)
        object_points_np = np.array(object_points, dtype=np.float64)

        object_points_np[:, [0, 2]] = object_points_np[:, [2, 0]] # swap X and Z

        save_camera_position(image_points_np, object_points_np, calibration_data_path, camera_position_data_output_path)

    # --- Alignment Visualization ---

    def show_alignment(self):
        """Warps Image B onto Image A using the calculated method and displays."""
        # If self.is_b_point_cloud is True, self.image_b will be the heatmap PIL image.
        warp_calculated = (self.homography_matrix is not None) or (self.tps_transformer is not None)
        if not warp_calculated:
            messagebox.showerror("Alignment Error", "Warp transform not calculated yet. Please calculate it first.")
            return

        try:
            # Calculate blended image only if needed
            if self.blended_pil_image is None:
                self.set_status("Warping and blending images...")
                self.root.update_idletasks()

                h_a, w_a = self.image_a.height, self.image_a.width
                # Ensure images are BGR for OpenCV
                img_a_cv = np.array(self.image_a.convert('RGB'))
                img_a_cv = cv2.cvtColor(img_a_cv, cv2.COLOR_RGB2BGR)
                img_b_cv = np.array(self.image_b.convert('RGB'))
                img_b_cv = cv2.cvtColor(img_b_cv, cv2.COLOR_RGB2BGR)

                # Warp Image B based on the calculated transform
                if self.warp_method.get() == WARP_METHOD_HOMOGRAPHY and self.homography_matrix is not None:
                    H_transform_to_A = self.homography_matrix # This is H_Bcoords_to_Acoords
                    if self.is_b_point_cloud and self.heatmap_yz_min_max_b:
                        y_min_data, y_max_data, z_min_data, z_max_data = self.heatmap_yz_min_max_b
                        delta_y = y_max_data - y_min_data
                        delta_z = z_max_data - z_min_data

                        # Matrix to transform heatmap pixel (hpx, hpy) to world (Yw, Zw)
                        # Yw = s_y_coeff * hpx + y_offset_coeff
                        # Zw = s_z_coeff * hpy + z_offset_coeff
                        s_y_coeff = delta_y / (HEATMAP_WIDTH - 1.0) if delta_y != 0 and HEATMAP_WIDTH > 1 else 0
                        y_offset_coeff = y_min_data

                        # Note: hpy is 0 at top (z_max_data), HEATMAP_HEIGHT-1 at bottom (z_min_data)
                        # Zw = z_max_data - hpy * (delta_z / (HEATMAP_HEIGHT - 1.0))
                        s_z_coeff = -delta_z / (HEATMAP_HEIGHT - 1.0) if delta_z != 0 and HEATMAP_HEIGHT > 1 else 0
                        z_offset_coeff = z_max_data
                        
                        if delta_y == 0: # All points have same Y
                            s_y_coeff = 0
                        if delta_z == 0: # All points have same Z
                            s_z_coeff = 0
                            z_offset_coeff = z_min_data # or z_max_data, they are equal

                        M_heatmap_px_to_world_yz = np.array([
                            [s_y_coeff, 0,           y_offset_coeff],
                            [0,         s_z_coeff,   z_offset_coeff],
                            [0,         0,           1             ]
                        ], dtype=np.float32)

                        # H_display transforms heatmap pixels to Image A pixels
                        H_display = H_transform_to_A @ M_heatmap_px_to_world_yz
                        warped_b = cv2.warpPerspective(img_b_cv, H_display, (w_a, h_a), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0,0,0))
                    else: # Image B is a regular image
                        warped_b = cv2.warpPerspective(img_b_cv, H_transform_to_A, (w_a, h_a), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0,0,0))

                elif self.warp_method.get() == WARP_METHOD_TPS and self.tps_transformer is not None:
                    # Verarbeite das Bild in kleineren Blöcken
                    block_size = 1000  # Verarbeite jeweils 1000 Punkte
                    h_a, w_a = img_a_cv.shape[:2]
                    
                    # Erstelle das vollständige Koordinatengitter
                    grid_y, grid_x = np.mgrid[0:h_a, 0:w_a].astype(np.float32)
                    
                    # map_x will store Y-coords in B's space, map_y will store Z-coords in B's space (or X,Y for regular image)
                    map_b_coord1 = np.zeros((h_a, w_a), dtype=np.float32) # For Y_world or X_pixel_B
                    map_b_coord2 = np.zeros((h_a, w_a), dtype=np.float32) # For Z_world or Y_pixel_B
                    
                    # Verarbeite das Gitter in Blöcken
                    for y in range(0, h_a, block_size):
                        y_end = min(y + block_size, h_a)
                        for x in range(0, w_a, block_size):
                            x_end = min(x + block_size, w_a)
                            
                            # Extrahiere den aktuellen Block
                            block_coords = []
                            for yi in range(y, y_end):
                                for xi in range(x, x_end):
                                    block_coords.append([xi, yi])
                            
                            block_coords = np.float32(block_coords).reshape(-1, 1, 2)
                            
                            # Wende TPS auf den Block an
                            try:
                                transformed_coords = self.tps_transformer.applyTransformation(block_coords)
                                
                                # Verarbeite die transformierten Koordinaten
                                if transformed_coords is not None:
                                    # Handle verschiedene mögliche Rückgabeformate
                                    if isinstance(transformed_coords, tuple):
                                        if len(transformed_coords) > 0:
                                            transformed_coords = transformed_coords[0]
                                    
                                    # Konvertiere zu NumPy-Array wenn nötig
                                    transformed_coords = np.array(transformed_coords, dtype=np.float32)
                                    
                                    # Fülle die Mapping-Arrays für diesen Block
                                    idx = 0
                                    for yi in range(y, y_end):
                                        for xi in range(x, x_end):
                                            if idx < len(transformed_coords): # Should always be true
                                                map_b_coord1[yi, xi] = transformed_coords[idx, 0]
                                                map_b_coord2[yi, xi] = transformed_coords[idx, 1]
                                                idx += 1
                            except Exception as e:
                                print(f"Fehler bei Block {x},{y}: {e}")
                    
                    if self.is_b_point_cloud and self.heatmap_yz_min_max_b:
                        # Convert world YZ maps (map_b_coord1=Y_world, map_b_coord2=Z_world) to heatmap pixel maps
                        y_min_data, y_max_data, z_min_data, z_max_data = self.heatmap_yz_min_max_b
                        delta_y = y_max_data - y_min_data
                        delta_z = z_max_data - z_min_data

                        if delta_y == 0 or HEATMAP_WIDTH <= 1:
                            map_x_heatmap_px = np.full_like(map_b_coord1, (HEATMAP_WIDTH - 1.0) / 2.0)
                        else:
                            map_x_heatmap_px = ((map_b_coord1 - y_min_data) / delta_y) * (HEATMAP_WIDTH - 1.0)

                        if delta_z == 0 or HEATMAP_HEIGHT <= 1:
                            map_y_heatmap_px = np.full_like(map_b_coord2, (HEATMAP_HEIGHT - 1.0) / 2.0)
                        else:
                            # map_b_coord2 contains Z_world. Heatmap Y pixel is (z_max - Z_world) / range * H
                            map_y_heatmap_px = ((z_max_data - map_b_coord2) / delta_z) * (HEATMAP_HEIGHT - 1.0)
                        
                        map_x_heatmap_px = np.clip(map_x_heatmap_px, 0, HEATMAP_WIDTH - 1).astype(np.float32)
                        map_y_heatmap_px = np.clip(map_y_heatmap_px, 0, HEATMAP_HEIGHT - 1).astype(np.float32)
                        
                        warped_b = cv2.remap(img_b_cv, map_x_heatmap_px, map_y_heatmap_px, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0,0,0))
                    else: # Image B is a regular image
                        # map_b_coord1 is X_pixel_B, map_b_coord2 is Y_pixel_B
                        warped_b = cv2.remap(img_b_cv, map_b_coord1.astype(np.float32), map_b_coord2.astype(np.float32), cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0,0,0))
                else:
                    messagebox.showerror("Alignment Error", "Warp transform not calculated or method mismatch.")
                    return


                # Ensure warped_b has the exact same dimensions as img_a_cv before blending
                if warped_b.shape != img_a_cv.shape:
                    print(f"Warning: Warped image shape {warped_b.shape} differs from target {img_a_cv.shape}. Resizing.")
                    warped_b = cv2.resize(warped_b, (img_a_cv.shape[1], img_a_cv.shape[0]), interpolation=cv2.INTER_LINEAR)

                blended_image_cv = cv2.addWeighted(img_a_cv, 0.5, warped_b, 0.5, 0)

                blended_image_rgb = cv2.cvtColor(blended_image_cv, cv2.COLOR_BGR2RGB)
                self.blended_pil_image = Image.fromarray(blended_image_rgb)
                self.set_status("Blending complete. Opening preview...")
                self.root.update_idletasks()
            else:
                # If blended_pil_image exists, but warp parameters might have changed,
                # we might need to re-evaluate if self.alignment_window_active_pc_slice_xyz
                # is still relevant or needs updating. For now, assume it's tied to the
                # state when the alignment window was first opened or warp last calculated.
                # If warp is recalculated, blended_pil_image becomes None, triggering re-blend.
                 self.set_status("Opening alignment preview...")

            # --- Create or Update Toplevel window ---
            # ... (Toplevel window creation/update logic remains the same) ...
            if not (self.alignment_window and self.alignment_window.winfo_exists()):
                self.alignment_window = Toplevel(self.root)
                self.alignment_window.title("Image Alignment Preview")
                self.alignment_window.protocol("WM_DELETE_WINDOW", self._on_alignment_close)
                self.alignment_point_pairs = [] # Clear points for new/reopened window

                # Reset selection index for alignment window
                self.alignment_current_selection_index = None

                # If Image B is a point cloud, store the active slice for nearest neighbor
                if self.is_b_point_cloud and self.last_filtered_points_for_heatmap_xyz is not None:
                    self.alignment_window_active_pc_slice_xyz = np.copy(self.last_filtered_points_for_heatmap_xyz)
                else:
                    self.alignment_window_active_pc_slice_xyz = None

                img_w, img_h = self.blended_pil_image.size
                # Adjust window width to accommodate listbox
                listbox_width_estimate = 250
                win_w, win_h = img_w + 40 + listbox_width_estimate, img_h + 80
                max_w = self.root.winfo_screenwidth() - 100
                max_h = self.root.winfo_screenheight() - 150
                win_w = min(win_w, max_w)
                win_h = min(win_h, max_h)
                self.alignment_window.geometry(f"{win_w}x{win_h}")

                # Main PanedWindow for resizable canvas and listbox
                alignment_paned_window = ttk.PanedWindow(self.alignment_window, orient=HORIZONTAL)
                alignment_paned_window.pack(fill=BOTH, expand=True, padx=10, pady=10)

                zoom_frame = Frame(self.alignment_window)
                zoom_frame.pack(pady=5)
                Label(zoom_frame, text="Zoom:").pack(side=LEFT, padx=5)
                Button(zoom_frame, text="+", width=2, command=lambda: self._zoom_alignment(ZOOM_STEP)).pack(side=LEFT)
                Button(zoom_frame, text="-", width=2, command=lambda: self._zoom_alignment(1/ZOOM_STEP)).pack(side=LEFT)
                Button(zoom_frame, text="100%", command=lambda: self._zoom_alignment(1.0, reset=True)).pack(side=LEFT, padx=5)

                # Canvas Frame (will be added to PanedWindow)
                canvas_container_frame = Frame(alignment_paned_window, width=img_w + 20) # Initial width hint
                alignment_paned_window.add(canvas_container_frame, weight=3) # Give more weight to canvas
                canvas_frame = Frame(canvas_container_frame, bd=1, relief=SUNKEN)
                canvas_frame.pack(fill=BOTH, expand=True) # Fill the container
                v_scroll = Scrollbar(canvas_frame, orient=VERTICAL)
                h_scroll = Scrollbar(canvas_frame, orient=HORIZONTAL)
                v_scroll.pack(side=RIGHT, fill=Y)
                h_scroll.pack(side=BOTTOM, fill=X)
                self.alignment_canvas = Canvas(canvas_frame, bg="gray",
                                               xscrollcommand=h_scroll.set,
                                               yscrollcommand=v_scroll.set, cursor="crosshair")
                self.alignment_canvas.pack(side=LEFT, fill=BOTH, expand=True)
                h_scroll.config(command=self.alignment_canvas.xview)
                v_scroll.config(command=self.alignment_canvas.yview)
                self.alignment_canvas.bind("<ButtonPress-2>", lambda e: self.alignment_canvas.scan_mark(e.x, e.y))
                self.alignment_canvas.bind("<B2-Motion>", lambda e: self.alignment_canvas.scan_dragto(e.x, e.y, gain=1))
                self.alignment_canvas.bind("<Button-1>", self._on_alignment_canvas_click)
                
                # Listbox and Controls Frame (will be added to PanedWindow)
                listbox_container_frame = Frame(alignment_paned_window, width=listbox_width_estimate)
                alignment_paned_window.add(listbox_container_frame, weight=1)

                alignment_list_frame = Frame(listbox_container_frame)
                alignment_list_frame.pack(fill=BOTH, expand=True) # Listbox takes most space here
                Label(alignment_list_frame, text="Alignment Points").pack()
                list_scrollbar = Scrollbar(alignment_list_frame, orient=VERTICAL)
                self.alignment_listbox = Listbox(alignment_list_frame, yscrollcommand=list_scrollbar.set, exportselection=False, selectmode=SINGLE)
                list_scrollbar.config(command=self.alignment_listbox.yview)
                list_scrollbar.pack(side=RIGHT, fill=Y)
                self.alignment_listbox.pack(side=LEFT, fill=BOTH, expand=True)
                self.alignment_listbox.bind("<<ListboxSelect>>", self._on_alignment_listbox_select)

                # Buttons below the listbox
                alignment_buttons_frame = Frame(listbox_container_frame)
                alignment_buttons_frame.pack(fill=X, pady=5)
                self.btn_remove_alignment_point = Button(alignment_buttons_frame, text="Remove Selected Point", command=self._remove_selected_alignment_point, state=DISABLED)
                self.btn_remove_alignment_point.pack(fill=X, pady=2)
                Button(alignment_buttons_frame, text="Clear All Points", command=self._clear_alignment_points).pack(fill=X, pady=2)
                Button(alignment_buttons_frame, text="Save Alignment Points", command=self._save_alignment_points).pack(fill=X, pady=2)

                self.alignment_zoom_level = 1.0

            self._update_alignment_display()
            self.alignment_window.lift()
            self.set_status("Alignment preview displayed.")

        except Exception as e:
            import traceback # Import traceback module here or at the top
            messagebox.showerror("Alignment Error", f"An error occurred during alignment visualization:\n{e}")
            traceback.print_exc() # Print traceback to console
            self.blended_pil_image = None
            if self.alignment_window and self.alignment_window.winfo_exists():
                self.alignment_window.destroy()
            self.alignment_window = None
            self.alignment_canvas = None
            self.tk_blended_image_display = None
            self.btn_remove_alignment_point = None
            self.alignment_listbox = None
            self.set_status("Alignment preview failed.")

    # ... (_on_alignment_close, _zoom_alignment, _update_alignment_display unchanged) ...
    def _on_alignment_close(self):
        """Cleanup when alignment window is closed."""
        if self.alignment_window:
            self.alignment_window.destroy()
        self.alignment_window = None
        self.alignment_canvas = None
        self.tk_blended_image_display = None
        self.btn_remove_alignment_point = None
        self.alignment_current_selection_index = None
        self.alignment_listbox = None
        # self.alignment_point_pairs are kept unless explicitly cleared or window is reopened
        self.alignment_window_active_pc_slice_xyz = None
        
    def _zoom_alignment(self, factor, reset=False):
        """Zooms the alignment preview window."""
        if not self.alignment_window or not self.alignment_window.winfo_exists() or not self.blended_pil_image:
            return

        current_zoom = self.alignment_zoom_level
        if reset:
            new_zoom = 1.0
        else:
            new_zoom = current_zoom * factor
            new_zoom = max(MIN_ZOOM, min(MAX_ZOOM, new_zoom))

        if new_zoom != current_zoom:
            self.alignment_zoom_level = new_zoom
            self._update_alignment_display()

    def _update_alignment_display(self):
        """Updates the alignment canvas image and scrollregion based on zoom."""
        if not self.alignment_canvas or not self.blended_pil_image:
            return

        zoom = self.alignment_zoom_level
        original_image = self.blended_pil_image

        new_width = int(original_image.width * zoom)
        new_height = int(original_image.height * zoom)

        if new_width < 1 or new_height < 1:
            self.alignment_canvas.delete("all")
            self.alignment_canvas.config(scrollregion=(0, 0, 0, 0))
            self.tk_blended_image_display = None
            return

        try:
            resample_filter = Image.Resampling.LANCZOS
            display_image = original_image.resize((new_width, new_height), resample_filter)

            self.tk_blended_image_display = ImageTk.PhotoImage(display_image)

            self.alignment_canvas.delete("all")
            self.alignment_canvas.create_image(0, 0, anchor=NW, image=self.tk_blended_image_display)
            self.alignment_canvas.config(scrollregion=(0, 0, new_width, new_height))
            self._redraw_alignment_points() # Redraw points on zoom

        except Exception as e:
            print(f"Error updating alignment display: {e}")
            messagebox.showwarning("Alignment Zoom Error", f"Could not resize alignment preview.\n{e}")
            self.alignment_canvas.delete("all")
            self.alignment_canvas.config(scrollregion=(0, 0, 0, 0))
            self.tk_blended_image_display = None

    def _redraw_alignment_points(self):
        if not self.alignment_canvas or not self.alignment_window.winfo_exists():
            return
        
        self.alignment_canvas.delete("alignment_point_marker")

        for i, (p_a, p_b_data) in enumerate(self.alignment_point_pairs):
            # p_a are the pixel coordinates on Image A (and thus on the blended image)
            # These need to be scaled by self.alignment_zoom_level for display on the alignment_canvas
            
            disp_ax_zoomed = p_a[0] * self.alignment_zoom_level
            disp_ay_zoomed = p_a[1] * self.alignment_zoom_level

            is_active = (i == self.alignment_current_selection_index)
            color = COLOR_ACTIVE if is_active else COLOR_INACTIVE
            
            self._draw_marker(self.alignment_canvas, disp_ax_zoomed, disp_ay_zoomed, color, "alignment_point_marker")


    def _on_alignment_canvas_click(self, event):
        if not self.alignment_canvas or not self.blended_pil_image:
            return

        # 1. Get click coordinates on the unzoomed blended image (Image A's coordinate system)
        canvas_x = self.alignment_canvas.canvasx(event.x)
        canvas_y = self.alignment_canvas.canvasy(event.y)
        img_a_px = int(canvas_x / self.alignment_zoom_level)
        img_a_py = int(canvas_y / self.alignment_zoom_level)

        # Ensure click is within bounds of Image A
        if not (0 <= img_a_px < self.image_a.width and 0 <= img_a_py < self.image_a.height):
            self.set_status("Alignment click outside image bounds.")
            return

        img_b_orig_x, img_b_orig_y = None, None

        # 2. Determine corresponding point in Image B's original space
        if self.warp_method.get() == WARP_METHOD_HOMOGRAPHY and self.homography_matrix is not None:
            try:
                H_inv = np.linalg.inv(self.homography_matrix)
                pt_A_homogeneous = np.array([img_a_px, img_a_py, 1.0], dtype=np.float32)
                transformed_pt_on_B_homogeneous = H_inv @ pt_A_homogeneous
                
                # Avoid division by zero if z-component is too small
                if abs(transformed_pt_on_B_homogeneous[2]) > 1e-9:
                    img_b_orig_x = transformed_pt_on_B_homogeneous[0] / transformed_pt_on_B_homogeneous[2]
                    img_b_orig_y = transformed_pt_on_B_homogeneous[1] / transformed_pt_on_B_homogeneous[2]
                else: # Should ideally not happen with valid homographies
                    print("Warning: Homogeneous z-coordinate near zero during inverse transform.")
            except np.linalg.LinAlgError:
                messagebox.showerror("Alignment Click Error", "Could not invert homography matrix.")
                return

        elif self.warp_method.get() == WARP_METHOD_TPS and self.tps_transformer is not None:
            pt_A_tps = np.float32([[img_a_px, img_a_py]]).reshape(-1, 1, 2)
            transformed_pt_on_B_tps_result = self.tps_transformer.applyTransformation(pt_A_tps)
            if transformed_pt_on_B_tps_result is not None and len(transformed_pt_on_B_tps_result[0]) > 0:
                 # Handle potential tuple output from older OpenCV versions
                coords_array = transformed_pt_on_B_tps_result[0] if isinstance(transformed_pt_on_B_tps_result, tuple) else transformed_pt_on_B_tps_result
                img_b_orig_x = coords_array[0,0,0]
                img_b_orig_y = coords_array[0,0,1]

        if img_b_orig_x is None or img_b_orig_y is None:
            self.set_status("Could not determine corresponding point on Image B.")
            return

        point_b_data_to_store = None
        point_b_display_str = ""

        if self.is_b_point_cloud:
            if self.heatmap_yz_min_max_b is None or self.alignment_window_active_pc_slice_xyz is None:
                self.set_status("Heatmap or PC slice data not ready for alignment click.")
                return

            world_coords = self._get_original_yz_from_heatmap_pixel(img_b_orig_x, img_b_orig_y)
            if world_coords is None:
                self.set_status("Could not convert heatmap pixel to world YZ for alignment.")
                return
            world_y, world_z = world_coords

            # Nearest Neighbor Search in the active PC slice (X,Y,Z)
            pc_slice = self.alignment_window_active_pc_slice_xyz # This is N x 3 (X,Y,Z)
            if pc_slice.shape[0] > 0:
                if SCIPY_AVAILABLE and KDTree is not None:
                    # Build KDTree on YZ coordinates of the slice
                    kdtree = KDTree(pc_slice[:, 1:3]) # Query YZ columns
                    distance, index = kdtree.query([world_y, world_z])
                    nearest_3d_point = pc_slice[index]
                else: # Fallback to brute force (slower)
                    distances_sq = np.sum((pc_slice[:, 1:3] - np.array([world_y, world_z]))**2, axis=1)
                    index = np.argmin(distances_sq)
                    nearest_3d_point = pc_slice[index]
                
                point_b_data_to_store = tuple(nearest_3d_point) # (X, Y, Z)
                point_b_display_str = f"PC({nearest_3d_point[0]:.2f}, {nearest_3d_point[1]:.2f}, {nearest_3d_point[2]:.2f})"
            else:
                self.set_status("No points in active PC slice for nearest neighbor.")
                return
        else: # Image B is a regular image
            point_b_data_to_store = (int(round(img_b_orig_x)), int(round(img_b_orig_y)))
            point_b_display_str = f"B({point_b_data_to_store[0]},{point_b_data_to_store[1]})"

        if point_b_data_to_store:
            self.alignment_point_pairs.append( ((img_a_px, img_a_py), point_b_data_to_store) )
            self._update_alignment_listbox()
            self.set_status(f"Alignment point added: A({img_a_px},{img_a_py}) <-> {point_b_display_str}")

    def _on_alignment_listbox_select(self, event):
        if not self.alignment_listbox or not self.alignment_listbox.curselection():
            if self.alignment_current_selection_index is not None:
                self.alignment_current_selection_index = None
                if self.btn_remove_alignment_point: self.btn_remove_alignment_point.config(state=DISABLED)
                self._redraw_alignment_points()
            return

        selected_indices = self.alignment_listbox.curselection()
        if selected_indices:
            self.alignment_current_selection_index = selected_indices[0]
            if self.btn_remove_alignment_point: self.btn_remove_alignment_point.config(state=NORMAL)
            self._redraw_alignment_points()

    def _update_alignment_listbox(self):
        if not self.alignment_listbox or not (self.alignment_window and self.alignment_window.winfo_exists()):
            return
        self.alignment_listbox.delete(0, END)
        for i, (p_a, p_b_data) in enumerate(self.alignment_point_pairs):
            pa_str = f"A({p_a[0]},{p_a[1]})"
            if self.is_b_point_cloud and len(p_b_data) == 3: # PC data (X,Y,Z)
                pb_str = f"PC({p_b_data[0]:.2f}, {p_b_data[1]:.2f}, {p_b_data[2]:.2f})"
            else: # Regular image B pixel data
                pb_str = f"B({p_b_data[0]},{p_b_data[1]})"
            self.alignment_listbox.insert(END, f"{i+1}: {pa_str} <-> {pb_str}")
        self.alignment_listbox.see(END)
        self._redraw_alignment_points() # Redraw points when listbox updates

    def _clear_alignment_points(self):
        if not self.alignment_window or not self.alignment_window.winfo_exists():
            return
        self.alignment_point_pairs = []
        self._update_alignment_listbox()
        self.set_status("Alignment points cleared.")
        self.alignment_current_selection_index = None
        if self.btn_remove_alignment_point: self.btn_remove_alignment_point.config(state=DISABLED)

    def _remove_selected_alignment_point(self):
        if self.alignment_current_selection_index is None or \
           not (0 <= self.alignment_current_selection_index < len(self.alignment_point_pairs)):
            return

        self.alignment_point_pairs.pop(self.alignment_current_selection_index)
        self.alignment_current_selection_index = None # Deselect
        self._update_alignment_listbox() # This will also call _redraw_alignment_points
        if self.btn_remove_alignment_point: self.btn_remove_alignment_point.config(state=DISABLED)
        self.set_status("Selected alignment point removed.")

    def _save_alignment_points(self):
        if not self.alignment_point_pairs:
            messagebox.showinfo("Save Alignment Points", "No alignment points to save.")
            return

        filepath = filedialog.asksaveasfilename(
            title="Save Alignment Points",
            defaultextension=".txt",
            filetypes=[("Text Files", "*.txt"), ("CSV Files", "*.csv"), ("All Files", "*.*")]
        )
        if not filepath: return

        try:
            with open(filepath, 'w') as f:
                f.write("# ImageA_X,ImageA_Y,ImageB_X_or_PC_X,ImageB_Y_or_PC_Y,(ImageB_Z_or_PC_Z if 3D)\n")
                for pa, pb_data in self.alignment_point_pairs:
                    f.write(f"{pa[0]},{pa[1]}," + ",".join(map(str, pb_data)) + "\n")
            self.set_status(f"Alignment points saved to {os.path.basename(filepath)}")
        except Exception as e:
            messagebox.showerror("Save Error", f"Failed to save alignment points:\n{e}")

# --- Main Execution ---
if __name__ == "__main__":
    root = tk.Tk()
    app = PointPairSelector(root)
    root.mainloop()
