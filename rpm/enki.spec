Name:           enki
# Update the following line to reflect the source release version you will be
# referencing below

# Update the following lines to reflect the source release version you will be
# referencing below
%global source_major 2
%global source_minor 0
%global source_patch 0
Version:        %{source_major}.%{source_minor}.%{source_patch}

# Update the following line with the git commit hash of the revision to use
# for example by running git show-ref -s --tags RELEASE_TAG
%global commit e138769516c8bd9aae51f7e71fd483c060615c2d
%global shortcommit %(c=%{commit}; echo ${c:0:7})

# Update the following line to set commit_is_tagged_as_source_release to 0 if
# and only if the commit hash is not from a git tag for an existing source
# release (i.e. it is a commit hash for a pre-release or post-release
# revision). Otherwise set it to 1.
%global commit_is_tagged_as_source_release 0
%if %{commit_is_tagged_as_source_release} == 0
  %global snapshot .%(date +%%Y%%m%%d)git%{shortcommit}
%endif

# Update the number(s) in the "Release:" line below as follows. If this is 
# the first RPM release for a particular source release version, then set the
# number to 0. If this is the first RPM pre-release for a future source
# release version (i.e. the "Version:" line above refers to a future
# source release version), then set the number to 0.0. Otherwise, leave the
# the number unchanged. It will get bumped when you run rpmdev-bumpspec.
Release:        0.4%{?snapshot}%{?dist}

Summary:        An open source robot simulator written in C++

%global lib_pkg_name lib%{name}%{source_major}

%if 0%{?suse_version}
%global buildoutdir build
%else
%global buildoutdir .
%endif

%if 0%{?suse_version}
License:        GPL-2.0
%else
License:        GPLv2
%endif
URL:            http://home.gna.org/enki/
Source0:        https://github.com/enki-community/enki/archive/%{commit}/%{name}-%{version}-%{shortcommit}.tar.gz
Patch0:         enki-rpm.patch

BuildRequires: SDL-devel
BuildRequires: binutils
BuildRequires: boost-devel
BuildRequires: cmake >= 2.8
BuildRequires: doxygen
BuildRequires: elfutils
BuildRequires: file
BuildRequires: gdb
BuildRequires: glibc-devel
BuildRequires: kernel-headers
BuildRequires: libstdc++-devel
%if 0%{?suse_version}
BuildRequires: Mesa-libGL-devel
BuildRequires: Mesa-libGLU-devel
%else
BuildRequires: mesa-libGL-devel
BuildRequires: mesa-libGLU-devel
%endif
BuildRequires: python-devel
BuildRequires: qt-devel
BuildRequires: gcc-c++

# sitelib for noarch packages, sitearch for others (remove the unneeded one)
%{!?python_sitelib: %global python_sitelib %(%{__python} -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())")}
%{!?python_sitearch: %global python_sitearch %(%{__python} -c "from distutils.sysconfig import get_python_lib; print(get_python_lib(1))")}

%description
No base package is installed.

%package -n python-py%{name}
Summary:        A module for using Enki from python
Group:          Development/Libraries/Python

%description  -n python-py%{name}
The python-py%{name} package contains a module for using Enki from python.

%package -n %{lib_pkg_name}
Summary:        An open source robot simulator written in C++
Group: System/Libraries

%description  -n %{lib_pkg_name}
Enki is an open source robot simulator written in C++. It provides collision
and limited physics support for robots evolving on a flat surface. On a
contemporary desktop computer, Enki is able to simulate groups of robots 
hundred times faster than real-time.

%package        devel
Summary:        Development files for %{name}
Requires:       python-py%{name} = %{version}-%{release}
Requires:       %{lib_pkg_name} = %{version}-%{release}
Group:          Development/Libraries/C and C++

%description    devel
The %{name}-devel package contains libraries, header files and docs for
developing applications that use %{name}.


%prep
%setup -q -n %{name}-%{commit}
%patch0 -p1

%build
%cmake
make %{?_smp_mflags}
doxygen %{_builddir}/%{buildsubdir}/Doxyfile

%install
rm -rf $RPM_BUILD_ROOT
cd %{buildoutdir}
make install DESTDIR=$RPM_BUILD_ROOT

%check
#ctest

%post -n %{lib_pkg_name}
/sbin/ldconfig

%postun -n %{lib_pkg_name}
/sbin/ldconfig


%files -n python-py%{name}
%doc LICENSE AUTHORS
# For arch-specific packages: sitearch
%{python_sitearch}/*

%files -n %{lib_pkg_name}
%doc LICENSE AUTHORS
%{_libdir}/*.so.*


%files devel
%doc %{buildoutdir}/doc/* examples
%{_includedir}/*
%{_libdir}/*.so


%changelog
* Tue Mar 04 2014 Dean Brettle <dean@brettle.com> - 2.0.0-0.4.20140303gite138769
- Updated spec to build on openSUSE and put libs in libenki2 package and python
  module in python-pyenki package.

* Sat Mar 01 2014 Dean Brettle <dean@brettle.com> - 2.0-0.3.20140228gite138769
- Changed SO_VERSION to SOVERSION so that libs only use major version number and
  added rpm directory with spec file and RPM building instructions.

* Thu Feb 27 2014 Dean Brettle <dean@brettle.com> - 2.0-0.2.20140223gite138769
- Moved shared lib version spec to top-level CMakeLists.txt

* Sun Feb 23 2014 Dean Brettle <dean@brettle.com> - 2.0-0.1.20140223gite138769
- Initial release
