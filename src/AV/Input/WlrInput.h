/*
Copyright (c) 2012-2017 Maarten Baert <maarten-baert@hotmail.com>

This file is part of SimpleScreenRecorder.

SimpleScreenRecorder is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

SimpleScreenRecorder is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with SimpleScreenRecorder.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#include "Global.h"

#include "SourceSink.h"
#include "MutexDataPair.h"

#include <wayland-client-protocol.h>
#include "wlr-screencopy-unstable-v1-client-protocol.h"
#include "xdg-output-unstable-v1-client-protocol.h"

class WlrInput : public VideoSource {

private:
	struct Rect {
		unsigned int m_x1, m_y1, m_x2, m_y2;
		inline Rect() {}
		inline Rect(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) : m_x1(x1), m_y1(y1), m_x2(x2), m_y2(y2) {}
	};
	struct SharedData {
		unsigned int m_current_width, m_current_height;
	};
	typedef MutexDataPair<SharedData>::Lock SharedLock;

    struct WlOutput {
        wl_output* output;
        zxdg_output_v1* zxdg_output;
        std::string name, description;
        int32_t x, y, width, height;
    };

    struct WlBuffer {
        struct wl_buffer *wl_buffer;
        void *data;
        enum wl_shm_format format;
        int width, height, stride;
        bool y_invert;

        timespec presented;
        uint32_t base_msec;
    };
private:
	unsigned int m_x, m_y, m_width, m_height;
	bool m_record_cursor, m_follow_cursor, m_follow_fullscreen;

	std::atomic<uint32_t> m_frame_counter;
	int64_t m_fps_last_timestamp;
	uint32_t m_fps_last_counter;
	double m_fps_current;

    wl_shm* m_shm;
    wl_registry* m_registry;
    wl_display* m_display;
    zxdg_output_manager_v1* m_xdg_output_manager;
    zwlr_screencopy_manager_v1* m_screencopy_manager;
    std::vector<WlOutput> m_outputs;
    WlOutput* m_current_output;

    WlBuffer m_buffer;
    bool m_buffer_copy_done;

	Rect m_screen_bbox;
	std::vector<Rect> m_screen_rects;
	std::vector<Rect> m_screen_dead_space;

	std::thread m_thread;
	MutexDataPair<SharedData> m_shared_data;
	std::atomic<bool> m_should_stop, m_error_occurred;

public:
	WlrInput(unsigned int x, unsigned int y, unsigned int width, unsigned int height, bool record_cursor, bool follow_cursor, bool follow_fullscreen);
	~WlrInput();

	// Reads the current size of the stream.
	// This function is thread-safe.
	void GetCurrentSize(unsigned int* width, unsigned int* height);

	// Returns the total number of captured frames.
	// This function is thread-safe.
	double GetFPS();

	// Returns whether an error has occurred in the input thread.
	// This function is thread-safe.
	inline bool HasErrorOccurred() { return m_error_occurred; }

private:
	void Init();
	void Free();

private:
	void InputThread();

    void CheckProtos();
    void SyncWayland();
    void LoadOutputInfo();

    AVPixelFormat GetInputFormat(WlBuffer& buf);
    static const zxdg_output_v1_listener XdgOutputImpl;
    static void HandleXdgOutputLogicalPosition(void* data, zxdg_output_v1* zxdg_output, int32_t x, int32_t y);
    static void HandleXdgOutputLogicalSize(void* data, zxdg_output_v1* zxdg_output, int32_t w, int32_t h);
    static void HandleXdgOutputDescription(void* data, zxdg_output_v1* zxdg_output_v1, const char* description);
    static void HandleXdgOutputName(void* data, zxdg_output_v1* zxdg_output_v1, const char* name);
    static void HandleXdgOutputDone(void* data, zxdg_output_v1* zxdg_output_v1);

    static const struct zwlr_screencopy_frame_v1_listener FrameListener;
    static void FrameHandleBuffer(void* data, struct zwlr_screencopy_frame_v1 *frame, uint32_t format, uint32_t width, uint32_t height, uint32_t stride);
    static void FrameHandleFlags(void* data, struct zwlr_screencopy_frame_v1 *, uint32_t flags);
    static void FrameHandleReady(void* data, struct zwlr_screencopy_frame_v1 *, uint32_t tv_sec_hi, uint32_t tv_sec_low, uint32_t tv_nsec);
    static void FrameHandleFailed(void* data, struct zwlr_screencopy_frame_v1 *);

    static const struct wl_registry_listener RegistryListener;
    static void HandleGlobal(void* data, struct wl_registry *registry, uint32_t name, const char *interface, uint32_t);
    static void HandleGlobalRemove(void*, struct wl_registry *, uint32_t);
};
