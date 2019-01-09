// Simple class to track cumulative / average execution
// time of a block of code

#pragma once

#include <chrono>
#include <map>
#include <string>

class TracerEntry
{
	public:
		TracerEntry()
			: count_(0)
			, total_time_(0.0)
			, started_(false)
	{
	}

	size_t count_;
	std::chrono::duration<double> total_time_;
	std::chrono::high_resolution_clock::time_point start_time_;
	bool started_;
};

class Tracer
{
	public:
		Tracer(const std::string name)
			:name_(name)
		{
		}

		// Mark the start of an event to time.  If the named event exists,
		// 
		void start(const std::string &label)
		{
			auto entry = map_.find(label);

			// If not found, create a new entry for this label
			if (entry == map_.end())
			{
				map_[label] = TracerEntry();
				entry = map_.find(label);
			}

			if (entry == map_.end())
			{
				ROS_ERROR_STREAM("Tracer::start : failed to insert label " << label);
				return;
			}

			if (entry->second.started_)
			{
				ROS_WARN_STREAM("Tracer::start : start called on already started label " << label);
				return;
			}
			entry->second.start_time_ = std::chrono::high_resolution_clock::now();
			entry->second.started_ = true;
		}

		void stop(const std::string &label)
		{
			auto entry = map_.find(label);

			// If not found, create a new entry for this label
			if (entry == map_.end())
			{
				ROS_ERROR_STREAM("Tracer::stop : couldn't find label " << label);
				return;
			}

			if (!entry->second.started_)
			{
				ROS_WARN_STREAM("Tracer::stop : label " << label << " not started");
				return;
			}
			entry->second.total_time_ += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - entry->second.start_time_);
			entry->second.count_ += 1;
			entry->second.started_ = false;
		}

		std::string report(const std::string &label) const
		{
			auto entry = map_.find(label);

			// If not found, create a new entry for this label
			if (entry == map_.end())
			{
				ROS_ERROR_STREAM("Tracer::report : couldn't find label " << label);
				return std::string();
			}
			std::stringstream s;
			const double avg_time = entry->second.total_time_ / std::chrono::duration<double>(entry->second.count_);
			s << name_ << ":" << label << " = " << avg_time;
			return s.str();
		}

		std::string report(void) const
		{
			std::stringstream s;
			s << name_ << ":" << std::endl;
			for (const auto &it : map_)
			{
				const double avg_time = it.second.total_time_ / std::chrono::duration<double>(it.second.count_);
				s << "\t" << it.first << " = " << avg_time << std::endl;
			}
			return s.str();
		}

	private:
		std::string name_;
		std::map<std::string, TracerEntry> map_;
};
