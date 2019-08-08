#ifndef __DB_ADS_H__
#define __DB_ADS_H__

#include <string>

#include <iostream>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp> 
#include <boost/interprocess/sync/interprocess_condition.hpp> 
#include <boost/interprocess/sync/scoped_lock.hpp> 
#include <boost/interprocess/sync/named_mutex.hpp> 
#include <boost/interprocess/sync/named_condition.hpp> 
#include <boost/interprocess/windows_shared_memory.hpp>

namespace dbAds
{
	namespace bi_space = boost::interprocess;

	template<class T, int size> class CSharedData
	{
	public:
		class CDataBlock
		{
		public:
			unsigned int m_dwSeqNum;
			unsigned int m_dwAmount;
			T data[size];

			inline unsigned int getCapacity()
			{
				return sizeof(data) / sizeof(data[0]);
			}

			CDataBlock& operator =(const CDataBlock& src)
			{
				if (this != &src)
				{
					this->m_dwSeqNum = src.m_dwSeqNum;
					this->m_dwAmount = src.m_dwAmount;
					if (this->m_dwAmount > 0)
					{
						memcpy(this->data, src.data, (&(src.data[src.m_dwAmount]) - &(src.data[0])));
					}
				}
				return *this;
			}
		};

	private:
		//bi_space::managed_shared_memory m_managed_shm;
		std::string m_szName;
		std::shared_ptr<bi_space::windows_shared_memory>	m_shdmem;
		std::shared_ptr<bi_space::mapped_region>			m_region;

	protected:
		std::shared_ptr<bi_space::named_mutex>     m_named_mtx;
		std::shared_ptr<bi_space::named_condition> m_named_cnd;
		CDataBlock *m_pointer = nullptr;

	public:
		CSharedData(std::string szName, bool bReadOnly = true)
		{
			m_szName = szName;
			if (!bReadOnly)
			{
				bi_space::named_mutex::remove((m_szName + "mtx").c_str());
				bi_space::named_condition::remove((m_szName + "cnd").c_str());

				m_shdmem = std::make_shared<bi_space::windows_shared_memory>(
					bi_space::open_or_create_t(),
					(m_szName + "_shm").c_str(),
					bi_space::read_write,
					sizeof(CDataBlock));

				m_region = std::make_shared<bi_space::mapped_region>(
					*m_shdmem,
					bi_space::read_write);

				m_named_mtx = std::make_shared<bi_space::named_mutex>(
					bi_space::open_or_create_t(),
					(m_szName + "mtx").c_str());

				m_named_cnd = std::make_shared<bi_space::named_condition>(
					bi_space::open_or_create_t(),
					(m_szName + "cnd").c_str());
			}
			else
			{
				m_shdmem = std::make_shared<bi_space::windows_shared_memory>(
					bi_space::open_only_t(),
					(m_szName + "_shm").c_str(),
					bi_space::read_only);

				m_region = std::make_shared<bi_space::mapped_region>(
					*m_shdmem,
					bi_space::read_only);

				m_named_mtx = std::make_shared<bi_space::named_mutex>(
					bi_space::open_only_t(),
					(m_szName + "mtx").c_str());

				m_named_cnd = std::make_shared<bi_space::named_condition>(
					bi_space::open_only_t(),
					(m_szName + "cnd").c_str());
			}

			m_pointer = reinterpret_cast<CDataBlock *>(m_region->get_address());
		}

		~CSharedData()
		{
			//bi_space::named_mutex::remove((m_szName + "mtx").c_str());
			//bi_space::named_condition::remove((m_szName + "cnd").c_str());
			//bi_space::windows_shared_memory::remove((m_szName + "_shm").c_str());
		}

		//if lpData->dwSeqNum==0, dwSeqNum will ++;
		int write(CDataBlock* lpData)
		{
			bi_space::scoped_lock<bi_space::named_mutex> lock(*m_named_mtx);
			if (m_pointer->getCapacity() > lpData->m_dwAmount)
			{
				unsigned int dwSeqNum = m_pointer->m_dwSeqNum;
				*m_pointer = *lpData;
				if (lpData->dwSeqNum == 0)
				{
					m_pointer->m_dwSeqNum = dwSeqNum++;
				}

				m_named_cnd->notify_all();
				return m_pointer->m_dwAmount;
			}
			return -1;
		}

		//Need fill the header of CDataBlock before call me 
		int read(CDataBlock* lpData)
		{
			bi_space::scoped_lock<bi_space::named_mutex> lock(*m_named_mtx);
			if (lpData->dwSeqNum != 0)
			{
				m_named_cnd->wait(lock, [](){return lpData->dwSeqNum == m_pointer->dwSeqNum});
			}
			else
			{
				m_named_cnd->wait(lock);
			}

			if (m_pointer->dwAmount <= lpData->m_dwAmount)
			{
				*lpData = *m_pointer;
				return m_pointer->m_dwAmount;
			}
			return -1;
		}

		void Lock()
		{
			return m_named_mtx->lock();
		}

		void Unlock()
		{
			return m_named_mtx->unlock();
		}

		operator const CDataBlock* ()
		{
			return m_pointer;
		}

		operator bool()
		{
			return (m_pointer != nullptr);
		}
	};
}

#endif
