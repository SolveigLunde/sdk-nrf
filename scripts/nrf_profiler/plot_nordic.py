#
# Copyright (c) 2018 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause

import matplotlib
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
from enum import Enum

import numpy as np
import sys
import time
import logging
import json

from processed_events import ProcessedEvents, EM_MEM_ADDRESS_DATA_DESC
from events import TrackedEvent, EventType
from stream import StreamError
from plot_nordic_config import PlotNordicConfig

class MouseButton(Enum):
    LEFT = 1
    MIDDLE = 2
    RIGHT = 3

class DrawState():
    def __init__(self, timeline_width_init,
                 event_processing_rect_height, event_submit_markersize):
        self.timeline_max = 0
        self.timeline_width = timeline_width_init

        self.event_processing_rect_height = event_processing_rect_height
        self.event_submit_markersize = event_submit_markersize

        self.y_max = None
        self.y_height = None

        self.l_line = None
        self.l_line_coord = None
        self.r_line = None
        self.r_line_coord = None
        self.duration_marker = None
        self.pan_x_start = None

        self.paused = False

        self.ax = None

        self.selected_event_submit = None
        self.selected_event_processing = None

        self.selected_event_text = None
        self.selected_event_textbox = None

        self.added_time = 0
        self.synchronized_with_events = False
        self.stale_events_displayed = False

class PlotNordic():

    def __init__(self, stream=None, event_close=None, log_lvl=logging.WARNING):
        plt.rcParams['toolbar'] = 'None'
        plt.ioff()
        self.plot_config = PlotNordicConfig
        self.draw_state = DrawState(
            self.plot_config['timeline_width_init'],
            self.plot_config['event_processing_rect_height'],
            self.plot_config['event_submit_markersize'])
        self.ani = None
        self.close_event_flag = False
        self.processed_events = ProcessedEvents()

        if stream is not None:
            timeouts = {
                'descriptions': 1,
                'events': 0
            }
            self.in_stream = stream
            self.in_stream.set_timeouts(timeouts)

        if event_close is not None:
            self.event_close = event_close

        self.logger = logging.getLogger('plot_nordic')
        self.logger_console = logging.StreamHandler()
        self.logger.setLevel(log_lvl)
        self.log_format = logging.Formatter(
            '[%(levelname)s] %(name)s: %(message)s')
        self.logger_console.setFormatter(self.log_format)
        self.logger.addHandler(self.logger_console)


    def read_data_from_files(self, events_filename, events_types_filename):
        self.processed_events.read_data_from_files(
            events_filename, events_types_filename)
        if not self.processed_events.verify():
            self.logger.warning("Missing event descriptions")

    def on_click_start_stop(self, event):
        if self.draw_state.paused:
            if self.draw_state.l_line is not None:
                self.draw_state.l_line.remove()
                self.draw_state.l_line = None
                self.draw_state.l_line_coord = None

            if self.draw_state.r_line is not None:
                self.draw_state.r_line.remove()
                self.draw_state.r_line = None
                self.draw_state.r_line_coord = None

            if self.draw_state.duration_marker is not None:
                self.draw_state.duration_marker.remove()

        self.draw_state.paused = not self.draw_state.paused

    def _prepare_plot(self, selected_events_types):

        self.draw_state.ax = plt.gca()
        self.draw_state.ax.set_navigate(False)

        fig = plt.gcf()
        fig.set_size_inches(
            self.plot_config['window_width_inch'],
            self.plot_config['window_height_inch'],
            forward=True)
        fig.canvas.draw()

        plt.xlabel("Time [s]")
        plt.title("Custom events")
        plt.grid(True)

        minimum = min(selected_events_types)
        maximum = max(selected_events_types)
        ticks = []
        labels = []
        for j in selected_events_types:
            ticks.append(j)
            labels.append(self.processed_events.registered_events_types[j].name)
        plt.yticks(ticks, labels)

        # min and max range of y axis are bigger by one so markers fit nicely
        # on plot
        self.draw_state.y_max = maximum + 1
        self.draw_state.y_height = maximum - minimum + 2
        plt.ylim([minimum - 1, maximum + 1])

        self.draw_state.selected_event_textbox = self.draw_state.ax.text(
            0.05,
            0.95,
            self.draw_state.selected_event_text,
            fontsize=10,
            transform=self.draw_state.ax.transAxes,
            verticalalignment='top',
            bbox=dict(
                boxstyle='round',
                alpha=0.5,
                facecolor='linen'))
        self.draw_state.selected_event_textbox.set_visible(False)

        fig.canvas.mpl_connect('scroll_event', self.scroll_event)
        fig.canvas.mpl_connect('button_press_event', self.button_press_event)
        fig.canvas.mpl_connect('button_release_event',
                               self.button_release_event)
        fig.canvas.mpl_connect('resize_event', PlotNordic.resize_event)
        fig.canvas.mpl_connect('close_event', self.close_event)

        plt.tight_layout()

        return fig

    def _get_relative_coords(self, event):
        # relative position of plot - x0, y0, width, height
        ax_loc = self.draw_state.ax.get_position().bounds
        window_size = plt.gcf().get_size_inches() * \
            plt.gcf().dpi  # window size - width, height
        x_rel = (event.x - ax_loc[0] * window_size[0]) \
                 / ax_loc[2] / window_size[0]
        y_rel = (event.y - ax_loc[1] * window_size[1]) \
                 / ax_loc[3] / window_size[1]
        return x_rel, y_rel

    def scroll_event(self, event):
        x_rel, _ = self._get_relative_coords(event)

        if event.button == 'up':
            if self.draw_state.paused:
                self.draw_state.timeline_max = self.draw_state.timeline_max - (1 - x_rel) * \
                    (self.draw_state.timeline_width - self.draw_state.timeline_width *
                     self.plot_config['timeline_scale_factor'])
            self.draw_state.timeline_width = self.draw_state.timeline_width * \
                self.plot_config['timeline_scale_factor']

        if event.button == 'down':
            if self.draw_state.paused:
                self.draw_state.timeline_max = self.draw_state.timeline_max + (1 - x_rel) * \
                    (self.draw_state.timeline_width / self.plot_config['timeline_scale_factor'] -
                     self.draw_state.timeline_width)
            self.draw_state.timeline_width = self.draw_state.timeline_width / \
                self.plot_config['timeline_scale_factor']

        self.draw_state.ax.set_xlim(
            self.draw_state.timeline_max -
            self.draw_state.timeline_width,
            self.draw_state.timeline_max)
        plt.draw()

    def _find_closest_event(self, x_coord, y_coord):
        filtered_id = list(filter(lambda x: x.submit.type_id == round(y_coord),
                                    self.processed_events.tracked_events))
        if len(filtered_id) == 0:
            return None
        if not self.processed_events.is_event_tracked(round(y_coord)):
            dists = list(map(lambda x: abs(x.submit.timestamp - x_coord), filtered_id))
            return filtered_id[np.argmin(dists)]
        else:
            matching_processing = list(
                filter(
                    lambda x: x.proc_start_time < x_coord < x.proc_end_time,
                    filtered_id))
            if matching_processing:
                return matching_processing[0]
            dists = list(map(lambda x: min([abs(x.submit.timestamp - x_coord),
                                abs(x.proc_start_time - x_coord), abs(x.proc_end_time - x_coord)]), filtered_id))
            return filtered_id[np.argmin(dists)]

    @staticmethod
    def _stringify_time(time_seconds):
        if time_seconds > 0.1:
            return '%.5f' % (time_seconds) + ' s'

        return '%.5f' % (1000 * time_seconds) + ' ms'

    def button_press_event(self, event):
        x_rel, y_rel = self._get_relative_coords(event)

        if event.button == MouseButton.LEFT.value:
            self.draw_state.pan_x_start1 = x_rel

        if event.button == MouseButton.MIDDLE.value:
            if self.draw_state.selected_event_submit is not None:
                for i in self.draw_state.selected_event_submit:
                    i.remove()
                self.draw_state.selected_event_submit = None

            if self.draw_state.selected_event_processing is not None:
                self.draw_state.selected_event_processing.remove()
                self.draw_state.selected_event_processing = None

            self.draw_state.selected_event_textbox.set_visible(False)

            if x_rel > 1 or x_rel < 0 or y_rel > 1 or y_rel < 0:
                plt.draw()
                return

            coord_x = self.draw_state.timeline_max - \
                (1 - x_rel) * self.draw_state.timeline_width
            coord_y = self.draw_state.y_max - \
                (1 - y_rel) * self.draw_state.y_height
            selected_event = self._find_closest_event(coord_x, coord_y)
            if selected_event is None:
                return
            event_submit = selected_event.submit

            self.draw_state.selected_event_submit = self.draw_state.ax.plot(
                event_submit.timestamp,
                event_submit.type_id,
                markersize=2*self.draw_state.event_submit_markersize,
                color='g',
                marker='o',
                linestyle=' ')

            if selected_event.proc_start_time is not None:
                self.draw_state.selected_event_processing = matplotlib.patches.Rectangle(
                    (selected_event.proc_start_time,
                    selected_event.submit.type_id -
                    self.draw_state.event_processing_rect_height),
                    selected_event.proc_end_time -
                    selected_event.proc_start_time,
                    2*self.draw_state.event_processing_rect_height,
                    color='g')
                self.draw_state.ax.add_artist(
                    self.draw_state.selected_event_processing)

            self.draw_state.selected_event_text = \
                self.processed_events.registered_events_types[event_submit.type_id].name + '\n'
            self.draw_state.selected_event_text += 'Submit: ' + \
                PlotNordic._stringify_time(event_submit.timestamp) + '\n'

            if selected_event.proc_start_time is not None:
                self.draw_state.selected_event_text += 'Processing start: ' + \
                    PlotNordic._stringify_time(
                        selected_event.proc_start_time) + '\n'
                self.draw_state.selected_event_text += 'Processing end: ' + \
                    PlotNordic._stringify_time(
                        selected_event.proc_end_time) + '\n'
                self.draw_state.selected_event_text += 'Processing time: ' + \
                    PlotNordic._stringify_time(selected_event.proc_end_time - \
                        selected_event.proc_start_time) + '\n'

            ev_type = self.processed_events.registered_events_types[event_submit.type_id]

            for i, data_desc in enumerate(ev_type.data_descriptions):
                if data_desc == EM_MEM_ADDRESS_DATA_DESC:
                    continue
                self.draw_state.selected_event_text += data_desc + ' = '
                self.draw_state.selected_event_text += str(event_submit.data[i]) + '\n'

            self.draw_state.selected_event_textbox.set_visible(True)
            self.draw_state.selected_event_textbox.set_text(
                self.draw_state.selected_event_text)

            plt.draw()

        if event.button == MouseButton.RIGHT.value:
            self.draw_state.pan_x_start2 = x_rel

    def button_release_event(self, event):
        x_rel, y_rel = self._get_relative_coords(event)

        if event.button == MouseButton.LEFT.value:
            if self.draw_state.paused:
                if abs(x_rel - self.draw_state.pan_x_start1) < 0.01:
                    if self.draw_state.l_line is not None:
                        self.draw_state.l_line.remove()
                        self.draw_state.l_line = None
                        self.draw_state.l_line_coord = None

                    if 0 <= x_rel <= 1:
                        if 0 <= y_rel <= 1:
                            self.draw_state.l_line_coord = self.draw_state.timeline_max - \
                                (1 - x_rel) * self.draw_state.timeline_width
                            self.draw_state.l_line = plt.axvline(
                                self.draw_state.l_line_coord)
                    plt.draw()

                else:
                    self.draw_state.timeline_max = self.draw_state.timeline_max - \
                        (x_rel - self.draw_state.pan_x_start1) * \
                        self.draw_state.timeline_width
                    self.draw_state.ax.set_xlim(
                        self.draw_state.timeline_max -
                        self.draw_state.timeline_width,
                        self.draw_state.timeline_max)
                    plt.draw()

        if event.button == MouseButton.RIGHT.value:
            if self.draw_state.paused:
                if abs(x_rel - self.draw_state.pan_x_start2) < 0.01:
                    if self.draw_state.r_line is not None:
                        self.draw_state.r_line.remove()
                        self.draw_state.r_line = None
                        self.draw_state.r_line_coord = None

                    if 0 <= x_rel <= 1:
                        if 0 <= y_rel <= 1:
                            self.draw_state.r_line_coord = self.draw_state.timeline_max - \
                                (1 - x_rel) * self.draw_state.timeline_width
                            self.draw_state.r_line = plt.axvline(
                                self.draw_state.r_line_coord, color='r')
                    plt.draw()

        if self.draw_state.r_line_coord is not None and self.draw_state.l_line_coord is not None:
            if self.draw_state.duration_marker is not None:
                self.draw_state.duration_marker.remove()
            bigger_coord = max(
                self.draw_state.r_line_coord,
                self.draw_state.l_line_coord)
            smaller_coord = min(
                self.draw_state.r_line_coord,
                self.draw_state.l_line_coord)
            self.draw_state.duration_marker = plt.annotate(
                text=PlotNordic._stringify_time(
                    bigger_coord - smaller_coord), xy=(
                    smaller_coord, 0.5), xytext=(
                    bigger_coord, 0.5), arrowprops=dict(
                    arrowstyle='<->'))
        else:
            if self.draw_state.duration_marker is not None:
                self.draw_state.duration_marker.remove()
                self.draw_state.duration_marker = None

    @staticmethod
    def resize_event(event):
        plt.tight_layout()

    def close_event(self, event):
        self.close_event_flag = True

    def animate_events_real_time(self, fig):
        rects = []
        events = []
        #Receive events
        while True:
            try:
                data = self.in_stream.recv_ev()
            except StreamError as err:
                if err.args[1] == StreamError.TIMEOUT_MSG:
                    break
                self.logger.error("Receiving error: {}. Exiting".format(err))
                self.close_event(None)
                sys.exit()
            data_str = data.decode()
            tracked_event = TrackedEvent.deserialize(data_str)

            events.append(tracked_event.submit)
            self.processed_events.tracked_events.append(tracked_event)

            if tracked_event.proc_start_time is not None:
                assert tracked_event.proc_end_time is not None
                rects.append(
                    matplotlib.patches.Rectangle(
                        (tracked_event.proc_start_time,
                            tracked_event.submit.type_id -
                            self.draw_state.event_processing_rect_height/2),
                        tracked_event.proc_end_time -
                        tracked_event.proc_start_time,
                        self.draw_state.event_processing_rect_height,
                        edgecolor='black'))

        # translating plot
        if not self.draw_state.synchronized_with_events:
            # ignore translating plot for stale events
            if not self.draw_state.stale_events_displayed:
                self.draw_state.stale_events_displayed = True
            else:
            # translate plot for new events
                if len(events) != 0:
                    self.draw_state.added_time = events[-1].timestamp - \
                                                   0.3 * self.draw_state.timeline_width
                    self.draw_state.synchronized_with_events = True

        if not self.draw_state.paused:
            self.draw_state.timeline_max = time.time() - self.start_time + \
                self.draw_state.added_time
            self.draw_state.ax.set_xlim(
                self.draw_state.timeline_max -
                self.draw_state.timeline_width,
                self.draw_state.timeline_max)

        # plotting events
        y = list(map(lambda x: x.type_id, events))
        x = list(map(lambda x: x.timestamp, events))
        self.draw_state.ax.plot(
            x,
            y,
            marker='o',
            linestyle=' ',
            color='r',
            markersize=self.draw_state.event_submit_markersize)

        self.draw_state.ax.add_collection(PatchCollection(rects))
        plt.gcf().canvas.flush_events()
        if self.event_close.is_set():
            self.close_event(None)
        if self.close_event_flag:
            sys.exit()

    def plot_events_real_time(self, selected_events_types=None):
        self.start_time = time.time()
        #Receive event descriptions
        while True:
            try:
                bytes = self.in_stream.recv_desc()
                break
            except StreamError as err:
                if err.args[1] == StreamError.TIMEOUT_MSG:
                    if self.event_close.is_set():
                        self.logger.info("Module closed before receiving event descriptions.")
                        sys.exit()
                    continue
                self.logger.error("Receiving error: {}. Exiting".format(err))
                sys.exit()
        data_str = bytes.decode()
        event_types_dict = json.loads(data_str)
        self.processed_events.registered_events_types = dict((int(k), EventType.deserialize(v))
                                                             for k, v in event_types_dict.items())
        if self.processed_events.registered_events_types is None:
            self.logger.error("Event descriptors not sent properly")
            sys.exit()
        if selected_events_types is None:
            selected_events_types = list(
                self.processed_events.registered_events_types.keys())

        fig = self._prepare_plot(selected_events_types)

        self.start_stop_ax = plt.axes([0.8, 0.025, 0.1, 0.04])
        self.start_stop_button = Button(self.start_stop_ax, 'Start/Stop')
        self.start_stop_button.on_clicked(self.on_click_start_stop)
        plt.sca(self.draw_state.ax)

        self.ani = animation.FuncAnimation(
            fig,
            self.animate_events_real_time,
            interval=self.plot_config['refresh_time'])
        plt.show()

    def plot_events_from_file(
            self, selected_events_types=None, one_line=False):
        self.draw_state.paused = True
        if len(self.processed_events.tracked_events) == 0 or \
                len(self.processed_events.registered_events_types) == 0:
            self.logger.error("Please read some events data before plotting")

        if selected_events_types is None:
            selected_events_types = list(
                self.processed_events.registered_events_types.keys())

        self._prepare_plot(selected_events_types)

        x = list(map(lambda x: x.submit.timestamp, self.processed_events.tracked_events))
        y = list(map(lambda x: x.submit.type_id, self.processed_events.tracked_events))
        self.draw_state.ax.plot(
            x,
            y,
            marker='o',
            linestyle=' ',
            color='r',
            markersize=self.draw_state.event_submit_markersize)

        rects = []
        for ev in self.processed_events.tracked_events:
            if ev.proc_start_time is None:
                continue
            rects.append(
                matplotlib.patches.Rectangle(
                    (ev.proc_start_time,
                        ev.submit.type_id - self.draw_state.event_processing_rect_height/2),
                        ev.proc_end_time - ev.proc_start_time,
                        self.draw_state.event_processing_rect_height,
                        edgecolor='black'))

        self.draw_state.ax.add_collection(PatchCollection(rects))

        self.draw_state.timeline_max = max(x) + 1
        self.draw_state.timeline_width = max(x) - min(x) + 2
        self.draw_state.ax.set_xlim([min(x) - 1, max(x) + 1])

        plt.draw()
        plt.show()
