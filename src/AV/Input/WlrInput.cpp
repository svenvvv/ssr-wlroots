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

#include "WlrInput.h"

#include "Logger.h"
#include "AVWrapper.h"
#include "Synchronizer.h"
#include "VideoEncoder.h"

#include "VideoPreviewer.h"

#include <unistd.h>
#include <sys/mman.h>
#include <string.h>

static int backingfile(off_t size) {
    char name[] = "/tmp/ssr-shared-XXXXXX";
    int fd = mkstemp(name);
    if (fd < 0) {
        return -1;
    }

    int status;
    while ((status = ftruncate(fd, size)) == EINTR);
    if (status < 0) {
        close(fd);
        return -1;
    }

    unlink(name);

    return fd;
}

static struct wl_buffer* create_shm_buffer(wl_shm* shm, uint32_t fmt, int width, int height, int stride, void** data_out) {
    int size = stride * height;

    int fd = backingfile(size);
    if (fd < 0) {
        return NULL;
    }

    void* data = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (data == MAP_FAILED) {
        close(fd);
        return NULL;
    }

    struct wl_shm_pool* pool = wl_shm_create_pool(shm, fd, size);
    struct wl_buffer* buffer = wl_shm_pool_create_buffer(pool, 0, width, height, stride, fmt);
    wl_shm_pool_destroy(pool);

    *data_out = data;
    close(fd);
    return buffer;
}

static uint64_t timespec_to_microsec(const timespec& ts) {
    return ts.tv_sec * 1000000ll + 1ll * ts.tv_nsec / 1000ll;
}

AVPixelFormat WlrInput::GetInputFormat(WlrInput::WlBuffer& buffer) {
    if (buffer.format == WL_SHM_FORMAT_ARGB8888 || buffer.format == WL_SHM_FORMAT_XRGB8888)
        return AV_PIX_FMT_BGR0;
    if (buffer.format == WL_SHM_FORMAT_XBGR8888 || buffer.format == WL_SHM_FORMAT_ABGR8888)
        return AV_PIX_FMT_RGB0;

    Logger::LogError("[WlrInput::GetInputFormat] " + Logger::tr("Unknown pixel format (%1)").arg(buffer.format));
    throw WlrException();
}

void WlrInput::HandleXdgOutputLogicalPosition(void* data, zxdg_output_v1* zxdg_output, int32_t x, int32_t y)
{
    WlrInput* self = (WlrInput*)data;
    for (auto& out : self->m_outputs) {
        if (out.zxdg_output == zxdg_output) {
            out.x = x;
            out.y = y;
        }
    }
}

void WlrInput::HandleXdgOutputLogicalSize(void* data, zxdg_output_v1* zxdg_output, int32_t width, int32_t height) {
    WlrInput* self = (WlrInput*)data;
    for (auto& wo : self->m_outputs) {
        if (wo.zxdg_output == zxdg_output) {
            wo.width = width;
            wo.height = height;
        }
    }
}

void WlrInput::HandleXdgOutputDescription(void* data, zxdg_output_v1* zxdg_output_v1, const char* description) {
    WlrInput* self = (WlrInput*)data;
    for (auto& out : self->m_outputs) {
        if (out.zxdg_output == zxdg_output_v1) {
            out.description = description;
        }
    }
}

void WlrInput::HandleXdgOutputName(void* data, zxdg_output_v1* zxdg_output_v1, const char* name) {
    WlrInput* self = (WlrInput*)data;
    for (auto& out : self->m_outputs) {
        if (out.zxdg_output == zxdg_output_v1)
            out.name = name;
    }
}

void WlrInput::HandleXdgOutputDone(void* data, zxdg_output_v1* zxdg_output_v1) {}

const zxdg_output_v1_listener WlrInput::XdgOutputImpl = {
    .logical_position = WlrInput::HandleXdgOutputLogicalPosition,
    .logical_size = WlrInput::HandleXdgOutputLogicalSize,
    .done = WlrInput::HandleXdgOutputDone,
    .name = WlrInput::HandleXdgOutputName,
    .description = WlrInput::HandleXdgOutputDescription
};

void WlrInput::FrameHandleBuffer(void* data, struct zwlr_screencopy_frame_v1*frame, uint32_t format, uint32_t width, uint32_t height, uint32_t stride) {
    WlrInput* self = (WlrInput*)data;
    auto& buf = self->m_buffer;

    buf.format = (wl_shm_format)format;
    buf.width = width;
    buf.height = height;
    buf.stride = stride;

    if (!buf.wl_buffer) {
        buf.wl_buffer = create_shm_buffer(self->m_shm, format, width, height, stride, &buf.data);
    }

    if (buf.wl_buffer == NULL) {
        Logger::LogError("[WlrInput::FrameHandleBuffer] " + Logger::tr("wl_buffer allocation failed!"));
    }
    zwlr_screencopy_frame_v1_copy(frame, buf.wl_buffer);
}

void WlrInput::FrameHandleFlags(void* data, struct zwlr_screencopy_frame_v1*, uint32_t flags) {
    WlrInput* self = (WlrInput*)data;
    self->m_buffer.y_invert = (flags & ZWLR_SCREENCOPY_FRAME_V1_FLAGS_Y_INVERT) != 0;
}

void WlrInput::FrameHandleReady(void* data, struct zwlr_screencopy_frame_v1*, uint32_t tv_sec_hi, uint32_t tv_sec_low, uint32_t tv_nsec) {
    WlrInput* self = (WlrInput*)data;
    self->m_buffer.presented.tv_sec = ((1ll * tv_sec_hi) << 32ll) | tv_sec_low;
    self->m_buffer.presented.tv_nsec = tv_nsec;
    self->m_buffer_copy_done = true;
}

void WlrInput::FrameHandleFailed(void* data, struct zwlr_screencopy_frame_v1*) {
    WlrInput* self = (WlrInput*)data;
    self->m_should_stop = true;
    Logger::LogError("[WlrInput::FrameHandleFailed] " + Logger::tr("Frame copy failed"));
    throw WlrException();
}

const struct zwlr_screencopy_frame_v1_listener WlrInput::FrameListener = {
    .buffer = WlrInput::FrameHandleBuffer,
    .flags = WlrInput::FrameHandleFlags,
    .ready = WlrInput::FrameHandleReady,
    .failed = WlrInput::FrameHandleFailed,
};

void WlrInput::HandleGlobal(void* data, struct wl_registry* registry, uint32_t name, const char* interface, uint32_t) {
    WlrInput* self = (WlrInput*)data;

    if (strcmp(interface, wl_output_interface.name) == 0) {
        auto output = (wl_output*)wl_registry_bind(registry, name, &wl_output_interface, 1);
        WlOutput out;
        out.output = output;
        self->m_outputs.push_back(out);
    } else if (strcmp(interface, wl_shm_interface.name) == 0) {
        self->m_shm = (wl_shm*)wl_registry_bind(registry, name, &wl_shm_interface, 1);
    } else if (strcmp(interface, zwlr_screencopy_manager_v1_interface.name) == 0) {
        self->m_screencopy_manager = (zwlr_screencopy_manager_v1*)wl_registry_bind(registry, name, &zwlr_screencopy_manager_v1_interface, 1);
    } else if (strcmp(interface, zxdg_output_manager_v1_interface.name) == 0) {
        self->m_xdg_output_manager = (zxdg_output_manager_v1*)wl_registry_bind(registry, name, &zxdg_output_manager_v1_interface, 2);
    }
}

void WlrInput::HandleGlobalRemove(void*, struct wl_registry*, uint32_t) {}

const struct wl_registry_listener WlrInput::RegistryListener = {
    .global = WlrInput::HandleGlobal,
    .global_remove = WlrInput::HandleGlobalRemove,
};

WlrInput::WlrInput(unsigned int x, unsigned int y, unsigned int width, unsigned int height, bool record_cursor, bool follow_cursor, bool follow_full_screen) {
	m_x = x;
	m_y = y;
	m_width = width;
	m_height = height;
	m_record_cursor = record_cursor;
	m_follow_cursor = follow_cursor;
	m_follow_fullscreen = follow_full_screen;

	m_shm = NULL;
	m_xdg_output_manager = NULL;
	m_screencopy_manager = NULL;

	m_screen_bbox = Rect(m_x, m_y, m_x + m_width, m_y + m_height);

	{
		SharedLock lock(&m_shared_data);
		lock->m_current_width = m_width;
		lock->m_current_height = m_height;
	}

	if(m_width == 0 || m_height == 0) {
		Logger::LogError("[WlrInput::Init] " + Logger::tr("Error: Width or height is zero!"));
		throw WlrException();
	}
	if(m_width > 10000 || m_height > 10000) {
		Logger::LogError("[WlrInput::Init] " + Logger::tr("Error: Width or height is too large, the maximum width and height is %1!").arg(10000));
		throw WlrException();
	}

	try {
		Init();
	} catch(...) {
		Free();
		throw;
	}

}

WlrInput::~WlrInput() {
	if (m_thread.joinable()) {
		Logger::LogInfo("[WlrInput::~WlrInput] " + Logger::tr("Stopping input thread ..."));
		m_should_stop = true;
		m_thread.join();
	}

	Free();
}

void WlrInput::GetCurrentSize(unsigned int* width, unsigned int *height) {
	SharedLock lock(&m_shared_data);
	*width = lock->m_current_width;
	*height = lock->m_current_height;
}

double WlrInput::GetFPS() {
	int64_t timestamp = hrt_time_micro();
	uint32_t frame_counter = m_frame_counter;
	unsigned int time = timestamp - m_fps_last_timestamp;
	if(time > 500000) {
		unsigned int frames = frame_counter - m_fps_last_counter;
		m_fps_last_timestamp = timestamp;
		m_fps_last_counter = frame_counter;
		m_fps_current = (double) frames / ((double) time * 1.0e-6);
	}
	return m_fps_current;
}

void WlrInput::Init() {
    m_display = wl_display_connect(NULL);
    if (m_display == NULL) {
        Logger::LogError("[WlrInput::Init] " + Logger::tr("Failed to connect to display."));
        throw WlrException();
    }

    m_registry = wl_display_get_registry(m_display);
    if (m_registry == NULL) {
        Logger::LogError("[WlrInput::Init] " + Logger::tr("Failed to get display registry."));
    }
    wl_registry_add_listener(m_registry, &RegistryListener, this);
    SyncWayland();

    CheckProtos();
    LoadOutputInfo();

    // TODO: Allow to select screen.
    assert(m_outputs.size() != 0);
    m_current_output = &m_outputs[0];

    m_buffer.wl_buffer = NULL;

	m_fps_last_timestamp = hrt_time_micro();
	m_fps_last_counter = 0;
	m_fps_current = 0.0;

	m_should_stop = false;
	m_error_occurred = false;
	m_thread = std::thread(&WlrInput::InputThread, this);
}

void WlrInput::Free() {
    wl_buffer_destroy(this->m_buffer.wl_buffer);
}

void WlrInput::InputThread() {
	try {
		Logger::LogInfo("[WlrInput::InputThread] " + Logger::tr("Input thread started."));

        int64_t last_timestamp = hrt_time_micro();
		while(!m_should_stop) {
			int64_t next_timestamp = CalculateNextVideoTimestamp();
            int64_t timestamp = hrt_time_micro();
			if(next_timestamp == SINK_TIMESTAMP_NONE) {
				usleep(20000);
				continue;
			} else if(next_timestamp != SINK_TIMESTAMP_ASAP) {
				int64_t wait = next_timestamp - timestamp;
				if(wait > 21000) {
					// the thread can't sleep for too long because it still has to check the m_should_stop flag periodically
					usleep(20000);
					continue;
				} else if(wait > 0) {
					usleep(wait);
                    timestamp = hrt_time_micro();
				}
			}

            m_buffer_copy_done = false;

            zwlr_screencopy_frame_v1* frame = zwlr_screencopy_manager_v1_capture_output_region(
                m_screencopy_manager, 1, m_current_output->output,
                m_x - m_current_output->x,
                m_y - m_current_output->y,
                m_width, m_height
            );

            zwlr_screencopy_frame_v1_add_listener(frame, &FrameListener, this);
            // Busy-wait for compositor to send frame to us.
            while (!m_buffer_copy_done && wl_display_dispatch(m_display) != -1);

			++m_frame_counter;

            auto& buf = m_buffer;

            const uint8_t* data = (uint8_t*)buf.data;
            if (buf.y_invert) {
                data += buf.stride * (buf.height - 1);
                buf.stride *= -1;
            }

            auto ts = timespec_to_microsec(buf.presented);
			PushVideoFrame(m_width, m_height, data, buf.stride, GetInputFormat(buf), ts);
            last_timestamp = ts;

            zwlr_screencopy_frame_v1_destroy(frame);
		}

		Logger::LogInfo("[WlrInput::InputThread] " + Logger::tr("Input thread stopped."));

	} catch(const std::exception& e) {
		m_error_occurred = true;
		Logger::LogError("[WlrInput::InputThread] " + Logger::tr("Exception '%1' in input thread.").arg(e.what()));
	} catch(...) {
		m_error_occurred = true;
		Logger::LogError("[WlrInput::InputThread] " + Logger::tr("Unknown exception in input thread."));
	}
}

void WlrInput::CheckProtos() {
    if (m_shm == NULL) {
		Logger::LogError("[WlrInput::CheckProtos] " + Logger::tr("Compositor doesn't supports wl_shm."));
        throw WlrException();
    }
    if (m_screencopy_manager == NULL) {
		Logger::LogError("[WlrInput::CheckProtos] " + Logger::tr("Compositor doesn't supports wlr-screencopy."));
        throw WlrException();
    }

    if (m_xdg_output_manager == NULL) {
		Logger::LogError("[WlrInput::CheckProtos] " + Logger::tr("Compositor doesn't supports xdg-object-manager."));
        throw WlrException();
    }

    if (m_outputs.empty()) {
		Logger::LogError("[WlrInput::CheckProtos] " + Logger::tr("No outputs reported by xdg-output."));
        throw WlrException();
    }
}

void WlrInput::SyncWayland() {
    wl_display_dispatch(m_display);
    wl_display_roundtrip(m_display);
}

void WlrInput::LoadOutputInfo() {
    for (auto& out : m_outputs) {
        out.zxdg_output = zxdg_output_manager_v1_get_xdg_output(m_xdg_output_manager, out.output);
        zxdg_output_v1_add_listener(out.zxdg_output, &XdgOutputImpl, this);
    }

    SyncWayland();
}
