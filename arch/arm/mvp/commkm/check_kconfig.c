/*
 * Linux 2.6.32 and later Kernel module for VMware MVP Guest Communications
 *
 * Copyright (C) 2010-2013 VMware, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; see the file COPYING.  If not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#line 5

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35)
#error "MVP requires a host kernel newer than 2.6.35"
#endif

#ifndef CONFIG_SYSFS
#error "MVP requires sysfs support (CONFIG_SYSFS)"
#endif

#ifndef CONFIG_NAMESPACES
#error "MVP networking support requires namespace support (CONFIG_NAMESPACES)"
#endif
#ifndef CONFIG_NET_NS
#error "MVP networking support requires Network Namespace support to be enabled (CONFIG_NET_NS)"
#endif

#ifndef CONFIG_INET
#error "MVP networking requires IPv4 support (CONFIG_INET)"
#endif
#ifndef CONFIG_IPV6
#error "MVP networking requires IPv6 support (CONFIG_IPV6)"
#endif

#if !defined(CONFIG_TUN)
#error "MVP VPN support requires TUN device support (CONFIG_TUN)"
#endif

#if !defined(CONFIG_NETFILTER) && !defined(PVTCP_DISABLE_NETFILTER)
#error "MVP networking support requires netfilter support (CONFIG_NETFILTER)"
#endif

#ifdef MVP_DEBUG
#if !defined(CONFIG_IKCONFIG) || !defined(CONFIG_IKCONFIG_PROC)
#error "MVP kernel /proc/config.gz support required for debuggability (CONFIG_IKCONFIG_PROC)"
#endif
#endif

#ifdef CONFIG_MIGRATION
#if defined(CONFIG_NUMA) || defined(CONFIG_CPUSETS) || defined(CONFIG_MEMORY_FAILURE)
#error "MVP not tested with migration features other than CONFIG_MEMORY_HOTPLUG and CONFIG_COMPACTION"
#endif
#endif
