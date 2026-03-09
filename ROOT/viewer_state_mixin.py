BASE_WIDGET_BINDINGS = (
    ("spin_base_x", "base_x"),
    ("spin_base_y", "base_y"),
    ("spin_base_z", "base_z"),
    ("spin_base_a", "base_a"),
    ("spin_base_b", "base_b"),
    ("spin_base_c", "base_c"),
)

TOOL_WIDGET_BINDINGS = (
    ("spin_tool_x", "tool_x"),
    ("spin_tool_y", "tool_y"),
    ("spin_tool_z", "tool_z"),
    ("spin_tool_a", "tool_a"),
    ("spin_tool_b", "tool_b"),
    ("spin_tool_c", "tool_c"),
)

TABLE_WIDGET_BINDINGS = (
    ("spin_table_x1", "table_x1"),
    ("spin_table_y1", "table_y1"),
    ("spin_table_x2", "table_x2"),
    ("spin_table_y2", "table_y2"),
)

EXTRUDER_WIDGET_BINDINGS = (
    ("spin_extruder_x", "extruder_x"),
    ("spin_extruder_y", "extruder_y"),
    ("spin_extruder_z", "extruder_z"),
    ("spin_extruder_a", "extruder_a"),
    ("spin_extruder_b", "extruder_b"),
    ("spin_extruder_c", "extruder_c"),
)

CHECKBOX_WIDGET_BINDINGS = (
    ("check_show_table", "show_table"),
    ("check_show_robot", "show_robot"),
    ("check_show_extruder", "show_extruder"),
)


class ViewerStateMixin:
    def _set_widgets_from_state(self, bindings):
        for widget_name, attr_name in bindings:
            widget = getattr(self, widget_name)
            widget.setValue(getattr(self, attr_name))

    def _set_checkboxes_from_state(self, bindings):
        for widget_name, attr_name in bindings:
            widget = getattr(self, widget_name)
            widget.setChecked(getattr(self, attr_name))

    def _set_state_from_widgets(self, bindings):
        for widget_name, attr_name in bindings:
            widget = getattr(self, widget_name)
            setattr(self, attr_name, widget.value())

    def _set_state_from_checkboxes(self, bindings):
        for widget_name, attr_name in bindings:
            widget = getattr(self, widget_name)
            setattr(self, attr_name, widget.isChecked())

    def _sync_ui_from_state(self):
        """Push all internal state values into UI widgets without triggering callbacks."""
        self.updating_sliders = True
        try:
            self.lbl_project_name.setText(f"\U0001F4C1 {self._project_name}")
            self.slider_thick.setValue(int(self.print_thickness * 10))
            self.lbl_thickness.setText(f"Extrusion Width:\n{self.print_thickness:.1f} mm")
            self._set_widgets_from_state(BASE_WIDGET_BINDINGS + TOOL_WIDGET_BINDINGS + TABLE_WIDGET_BINDINGS + EXTRUDER_WIDGET_BINDINGS)
            self._set_checkboxes_from_state(CHECKBOX_WIDGET_BINDINGS)

            ik_idx = self.combo_ik_config.findText(self.ik_config)
            if ik_idx >= 0:
                self.combo_ik_config.setCurrentIndex(ik_idx)

            ext_idx = self.combo_extruder_stl.findText(self.extruder_stl)
            if ext_idx >= 0:
                self.combo_extruder_stl.setCurrentIndex(ext_idx)

            self._select_plugin_by_file_name(self.combo_postprocessor, self.last_postprocessor)
        finally:
            self.updating_sliders = False

    def draw_table(self):
        """Draw or remove the build-plate rectangle in the 3D scene."""
        if self.table_actor:
            self.plotter.remove_actor(self.table_actor)
            self.table_actor = None

        if self.show_table:
            width = abs(self.table_x2 - self.table_x1)
            height = abs(self.table_y2 - self.table_y1)
            if width > 0 and height > 0:
                center_x = (self.table_x1 + self.table_x2) / 2.0
                center_y = (self.table_y1 + self.table_y2) / 2.0
                plane = self.pv.Plane(
                    center=(center_x, center_y, -0.5),
                    direction=(0, 0, 1),
                    i_size=width,
                    j_size=height,
                )
                self.table_actor = self.plotter.add_mesh(
                    plane,
                    color="#303030",
                    opacity=0.8,
                    show_edges=True,
                    edge_color="black",
                    reset_camera=False,
                )

        self.plotter.render()

    def on_table_changed(self):
        """Handle table visibility or dimension changes."""
        if self.updating_sliders:
            return
        self._set_state_from_checkboxes((CHECKBOX_WIDGET_BINDINGS[0],))
        self._set_state_from_widgets(TABLE_WIDGET_BINDINGS)
        self.draw_table()
        self.save_settings()

    def on_robot_changed(self):
        """Toggle robot mesh visibility."""
        if self.updating_sliders:
            return
        self._set_state_from_checkboxes((CHECKBOX_WIDGET_BINDINGS[1],))
        for actor in self.robot_actors.values():
            actor.SetVisibility(self.show_robot)
        self.plotter.render()
        self.save_settings()

    def on_transform_changed(self):
        """Handle base/tool/extruder transform spinbox changes."""
        if self.updating_sliders:
            return
        self._set_state_from_widgets(BASE_WIDGET_BINDINGS + TOOL_WIDGET_BINDINGS + EXTRUDER_WIDGET_BINDINGS)
        self.save_settings()

        if self.points_xyz is not None:
            self._request_update()

    def on_extruder_changed(self):
        """Handle extruder STL selection or visibility toggle."""
        if self.updating_sliders:
            return
        self.extruder_stl = self.combo_extruder_stl.currentText()
        self._set_state_from_checkboxes((CHECKBOX_WIDGET_BINDINGS[2],))
        self.save_settings()
        self._reload_extruder_mesh()
        if self.points_xyz is not None:
            self._request_update()

    def on_ik_config_changed(self, text):
        """Handle change in IK initial configuration."""
        if getattr(self, "updating_sliders", False):
            return
        self.ik_config = text
        self._force_ik_seed_from_config = True
        self.save_settings()
        if self.points_xyz is not None:
            self._request_update()
