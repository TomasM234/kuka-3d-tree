import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import platform
import threading
import numpy as np

import matplotlib
# MacOS needs a specific backend, but we're on Windows so TkAgg is fine.
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection

class RobotPathViewer:
    def __init__(self, root):
        self.root = root
        self.root.title("Universal Robot CSV 3D Viewer")
        self.root.geometry("1200x900")
        
        # Will store: [(type, x, y, z, e_ratio, layer_idx), ...]
        self.data_points = []
        # Will store mapping from layer_idx -> max_point_index_in_that_layer
        self.layer_end_indices = {}
        
        self.original_csv_path = ""
        self.current_step = 0
        self.current_layer = 0
        self.max_layer = 0
        
        self.setup_ui()
        
    def setup_ui(self):
        # Control Frame (Top)
        self.ctrl_frame = ttk.Frame(self.root, padding="10")
        self.ctrl_frame.pack(side=tk.TOP, fill=tk.X)
        
        self.btn_open = ttk.Button(self.ctrl_frame, text="Open CSV File...", command=self.load_file)
        self.btn_open.pack(side=tk.LEFT, padx=5)
        
        self.lbl_status = ttk.Label(self.ctrl_frame, text="No file loaded.")
        self.lbl_status.pack(side=tk.LEFT, padx=20)
        
        # Main content area 
        self.main_frame = ttk.Frame(self.root)
        self.main_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
        # Vertical Timeline (Right)
        self.v_timeline_frame = ttk.Frame(self.main_frame, padding="10")
        self.v_timeline_frame.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.lbl_layer = ttk.Label(self.v_timeline_frame, text="Layer: 0 / 0")
        self.lbl_layer.pack(side=tk.TOP, pady=(0, 5))
        
        self.v_slider = ttk.Scale(
            self.v_timeline_frame, 
            from_=0, 
            to=0, 
            orient=tk.VERTICAL,
            command=self.on_v_slider_change
        )
        self.v_slider.pack(side=tk.TOP, fill=tk.Y, expand=True)
        self.v_slider.state(['disabled'])
        
        # Plot Frame (Middle/Left)
        self.plot_frame = ttk.Frame(self.main_frame)
        self.plot_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Timeline Frame (Bottom Slider)
        self.timeline_frame = ttk.Frame(self.root, padding="10")
        self.timeline_frame.pack(side=tk.BOTTOM, fill=tk.X)
        
        self.lbl_progress = ttk.Label(self.timeline_frame, text="Progress: 0 / 0 points")
        self.lbl_progress.pack(side=tk.TOP, pady=(0, 5))
        
        self.slider = ttk.Scale(
            self.timeline_frame, 
            from_=0, 
            to=0, 
            orient=tk.HORIZONTAL,
            command=self.on_slider_change
        )
        self.slider.pack(side=tk.TOP, fill=tk.X, expand=True)
        self.slider.state(['disabled'])
        
        # Setup Figure
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Setup Canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
        # Setup Toolbar
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.plot_frame)
        self.toolbar.update()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
        self._format_axes()

    def on_v_slider_change(self, val):
        """Layer Slider (Vertical) changed"""
        if not self.data_points or getattr(self, '_updating_sliders', False):
            return
            
        layer = int(float(val))
        self.current_layer = layer
        self.lbl_layer.config(text=f"Layer: {layer} / {self.max_layer}")
        
        target_step = self.layer_end_indices.get(layer, 0)
        
        self._updating_sliders = True
        self.slider.set(target_step)
        self._updating_sliders = False
        
        self.current_step = target_step
        self.lbl_progress.config(text=f"Progress: {target_step} / {len(self.data_points)} points")
        
        if hasattr(self, '_redraw_job') and self._redraw_job is not None:
            self.root.after_cancel(self._redraw_job)
        self._redraw_job = self.root.after(50, self.plot_data_fast)

    def on_slider_change(self, val):
        """Timeline Slider (Horizontal) changed"""
        if not self.data_points or getattr(self, '_updating_sliders', False):
            return
        
        step = int(float(val))
        self.current_step = step
        self.lbl_progress.config(text=f"Progress: {step} / {len(self.data_points)} points")
        
        if step > 0 and step <= len(self.data_points):
            current_layer_idx = self.data_points[step-1][5]
            self.current_layer = current_layer_idx
            self.lbl_layer.config(text=f"Layer: {current_layer_idx} / {self.max_layer}")
            
            self._updating_sliders = True
            self.v_slider.set(current_layer_idx)
            self._updating_sliders = False
            
        if hasattr(self, '_redraw_job') and self._redraw_job is not None:
            self.root.after_cancel(self._redraw_job)
        self._redraw_job = self.root.after(50, self.plot_data_fast)

    def _format_axes(self):
        self.ax.set_title("Robot 3D Path Visualization")
        self.ax.xaxis.pane.fill = False
        self.ax.yaxis.pane.fill = False
        self.ax.zaxis.pane.fill = False
        self.ax.xaxis.pane.set_edgecolor('w')
        self.ax.yaxis.pane.set_edgecolor('w')
        self.ax.zaxis.pane.set_edgecolor('w')
        self.ax.grid(False)

    def load_file(self):
        file_path = filedialog.askopenfilename(
            title="Select Robot Data Stream",
            filetypes=(("CSV files", "*.csv"), ("All files", "*.*"))
        )
        if not file_path:
            return
            
        self.original_csv_path = file_path
        self.lbl_status.config(text=f"Loading {file_path.split('/')[-1]}...")
        
        self.slider.state(['disabled'])
        self.v_slider.state(['disabled'])
        self.root.update_idletasks()
        
        threading.Thread(target=self.parse_and_plot, daemon=True).start()

    def parse_and_plot(self):
        try:
            points = []
            layer_ends = {}
            current_layer = 0
            
            with open(self.original_csv_path, 'r', encoding='utf-8') as f:
                for line_idx, line in enumerate(f):
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                        
                    parts = line.split(';')
                    # # TYPE;X;Y;Z;TCP_SPEED;E_RATIO;TEMP;FAN_PCT;LAYER;FEATURE;PROGRESS
                    # LAYER is at index 8
                    if len(parts) >= 9:
                        try:
                            p_type = parts[0]
                            x = float(parts[1])
                            y = float(parts[2])
                            z = float(parts[3])
                            e_ratio = float(parts[5])
                            layer_idx = int(parts[8])
                            
                            points.append((p_type, x, y, z, e_ratio, layer_idx))
                            
                            # Update layer mapping to always hold the highest point index for any given layer
                            # (Since points append 1 by 1, the last time we see layer `n`, `len(points)` is its end)
                            layer_ends[layer_idx] = len(points)
                            current_layer = max(current_layer, layer_idx)
                            
                        except ValueError:
                            pass
            
            self.data_points = points
            self.layer_end_indices = layer_ends
            self.max_layer = current_layer
            
            if not self.data_points:
                self.root.after(0, lambda: messagebox.showerror("Error", "No valid data points found in CSV."))
                self.root.after(0, lambda: self.lbl_status.config(text="Load failed."))
                return
                
            self.root.after(0, self.setup_new_plot)
            
        except Exception as e:
            self.root.after(0, lambda: messagebox.showerror("Error Loading File", str(e)))

    def _draw_build_plate(self):
        bp_x = [0, 550, 550, 0, 0]
        bp_y = [0, 0, 650, 650, 0]
        bp_z = [0, 0, 0, 0, 0]
        self.ax.plot(bp_x, bp_y, bp_z, color='black', linewidth=1.5, alpha=0.5)

    def _set_equal_aspect_3d(self, ax):
        x_limits = ax.get_xlim3d()
        y_limits = ax.get_ylim3d()
        z_limits = ax.get_zlim3d()

        x_range = abs(x_limits[1] - x_limits[0])
        x_middle = np.mean(x_limits)
        y_range = abs(y_limits[1] - y_limits[0])
        y_middle = np.mean(y_limits)
        z_range = abs(z_limits[1] - z_limits[0])
        z_middle = np.mean(z_limits)

        plot_radius = 0.5*max([x_range, y_range, z_range])

        ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
        ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
        ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

    def setup_new_plot(self):
        self.lbl_status.config(text="Preparing Scene...")
        
        num_points = len(self.data_points)
        
        # Setup Horizontal Slider
        self.slider.config(to=num_points)
        self.slider.state(['!disabled'])
        self.slider.set(num_points) 
        self.current_step = num_points
        
        # Setup Vertical Layer Slider
        # To make it natural (dragging UP = higher layer), invert from/to
        self.v_slider.config(from_=self.max_layer, to=0)
        self.v_slider.state(['!disabled'])
        self.v_slider.set(self.max_layer)
        self.current_layer = self.max_layer
        
        self.plot_data_fast(initial=True)

    def plot_data_fast(self, initial=False):
        if initial:
            self.ax.clear()
            self._format_axes()
            
            self.ax.set_title(f"Visualizing: {self.original_csv_path.split('/')[-1]}")
            self.ax.set_xlabel('X [mm]')
            self.ax.set_ylabel('Y [mm]')
            self.ax.set_zlabel('Z [mm]')
            
            self.ax.quiver(0, 0, 0, 50, 0, 0, color='r', arrow_length_ratio=0.1, label='X')
            self.ax.quiver(0, 0, 0, 0, 50, 0, color='g', arrow_length_ratio=0.1, label='Y')
            self.ax.quiver(0, 0, 0, 0, 0, 50, color='b', arrow_length_ratio=0.1, label='Z')
            
            self._draw_build_plate()
            
            all_x = [p[1] for p in self.data_points] + [0, 550]
            all_y = [p[2] for p in self.data_points] + [0, 650]
            all_z = [p[3] for p in self.data_points] + [0]
            
            self.ax.set_xlim3d(min(all_x), max(all_x))
            self.ax.set_ylim3d(min(all_y), max(all_y))
            self.ax.set_zlim3d(min(all_z), max(all_z))
            self._set_equal_aspect_3d(self.ax)
            self.ax.view_init(elev=30., azim=-45)

        for collection in list(self.ax.collections):
            collection.remove()

        if self.current_step <= 0:
            self.canvas.draw_idle()
            return
            
        segments = []
        colors = []
        prev_pt = None
        
        # We only plot points up to self.current_step
        # Even if vertical slider changes, it drives current_step!
        visible_points = self.data_points[:self.current_step]
        
        for p_type, x, y, z, e_ratio, l_idx in visible_points:
            curr_pt = (x, y, z)
            
            if prev_pt is not None:
                segments.append([prev_pt, curr_pt])
                c = 'gray'
                if p_type == 'P' and e_ratio > 0:
                    c = 'green'
                elif p_type == 'R' or p_type == 'U' or (p_type == 'P' and e_ratio < 0):
                    c = 'red'
                colors.append(c)
            
            prev_pt = curr_pt
            
        if segments:
            lc = Line3DCollection(segments, colors=colors, linewidths=1.0, alpha=0.8)
            self.ax.add_collection3d(lc)
            last_x, last_y, last_z = curr_pt
            self.ax.scatter(last_x, last_y, last_z, color='cyan', s=30, zorder=10)

        self.canvas.draw_idle() 
        
        if initial:
            self.lbl_status.config(text=f"Loaded {len(self.data_points)} pts. Print(+E): Green, Travel: Grey, Ret/Unret: Red")


if __name__ == "__main__":
    root = tk.Tk()
    app = RobotPathViewer(root)
    root.mainloop()
