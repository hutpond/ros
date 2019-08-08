#pragma once
#include <memory>
#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

//#include "libAdsTemplate.h"
#include "DBAds.h"

namespace dbAds
{
	typedef struct
	{
		uint32_t wTimeStamp;
		uint8_t ucLine;
		uint8_t ucIntensity;
	}PointTLI;

	typedef std::shared_ptr<std::vector<PointTLI>> PointTLIPtr;

	typedef struct
	{
		uint32_t wTimeStamp;
		float x;
		float y;
		float z;
		uint8_t ucLine;
		uint8_t ucIntensity;
	}LidarData;

	class CLidarSharedData : public CSharedData<LidarData, 16 * 2000 * 4>
	{
	public:
		CLidarSharedData(std::string szName, bool bReadOnly = true) 
			: CSharedData<LidarData, 16 * 2000 * 4>(szName, bReadOnly)
		{

		}

		int write(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzPtr, PointTLIPtr tliPtr)
		{
			if (xyzPtr->points.size() != tliPtr->size())
			{
				return -1;
			}

			if (xyzPtr->points.size() > m_pointer->getCapacity())
			{
				return -1;
			}

			boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_named_mtx);
			for (int i = 0; i < xyzPtr->points.size(); i++)
			{
				m_pointer->data[i].wTimeStamp = tliPtr->at(i).wTimeStamp;
				m_pointer->data[i].x = xyzPtr->points[i].x;
				m_pointer->data[i].y = xyzPtr->points[i].y;
				m_pointer->data[i].z = xyzPtr->points[i].z;
				m_pointer->data[i].ucLine = tliPtr->at(i).ucLine;
				m_pointer->data[i].ucIntensity = tliPtr->at(i).ucIntensity;
			}
			m_pointer->m_dwAmount = xyzPtr->points.size();
			m_pointer->m_dwSeqNum++;
			
			m_named_cnd->notify_all();
			return xyzPtr->points.size();
		}

		int read(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzPtr, PointTLIPtr tliPtr)
		{
			boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*m_named_mtx);
			m_named_cnd->wait(lock);

			if (tliPtr)
			{
				for (int i = 0; i < m_pointer->m_dwAmount; i++)
				{
					xyzPtr->push_back(pcl::PointXYZ(
						m_pointer->data[i].x,
						m_pointer->data[i].y,
						m_pointer->data[i].z)
						);

					PointTLI tli;
					tli.wTimeStamp = m_pointer->data[i].wTimeStamp;
					tli.ucIntensity = m_pointer->data[i].ucIntensity;
					tli.ucLine = m_pointer->data[i].ucLine;
					tliPtr->push_back(tli);
				}
			}
			else
			{
				for (int i = 0; i < m_pointer->m_dwAmount; i++)
				{
					xyzPtr->push_back(pcl::PointXYZ(
						m_pointer->data[i].x,
						m_pointer->data[i].y,
						m_pointer->data[i].z)
						);
				}
			}
			return m_pointer->m_dwAmount;
		}
	};
}
