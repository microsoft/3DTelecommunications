#pragma once

#include <string>

#include "asio.hpp"

#include <thread>
#include <mutex>
#include <condition_variable>

#include "data_format.h"

using namespace boost;

class abstract_loader
{
public:
	abstract_loader() {}
	virtual ~abstract_loader() {}

	virtual void read_frame(int frame) = 0;
	virtual int frame_count() const = 0;
	virtual int cached_frame_count() const = 0;
};
