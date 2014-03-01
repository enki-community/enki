Name:           enki
# Update the following line to reflect the source release version you will be
# referencing below

Version:        2.0

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
Release:        0.3%{?snapshot}%{?dist}

Summary:        An open source robot simulator written in C++

License:        GPLv2
URL:            http://home.gna.org/enki/
Source0:        https://github.com/enki-community/enki/archive/%{commit}/%{name}-%{version}-%{shortcommit}.tar.gz
Patch0:         enki-rpm.patch

BuildRequires: SDL-devel
BuildRequires: binutils
BuildRequires: boost-devel
BuildRequires: ccache
BuildRequires: cmake
BuildRequires: doxygen
BuildRequires: dwz
BuildRequires: elfutils
BuildRequires: file
BuildRequires: gdb
BuildRequires: glibc-devel
BuildRequires: glibc-headers
BuildRequires: kernel-headers
BuildRequires: libstdc++-devel
BuildRequires: mesa-libGL-devel
BuildRequires: mesa-libGLU-devel
BuildRequires: python-devel
BuildRequires: python-libs
BuildRequires: qt-devel

# sitelib for noarch packages, sitearch for others (remove the unneeded one)
%{!?python_sitelib: %global python_sitelib %(%{__python} -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())")}
%{!?python_sitearch: %global python_sitearch %(%{__python} -c "from distutils.sysconfig import get_python_lib; print(get_python_lib(1))")}


%description
Enki is an open source robot simulator written in C++. It provides collision
and limited physics support for robots evolving on a flat surface. On a
contemporary desktop computer, Enki is able to simulate groups of robots 
hundred times faster than real-time.

%package        devel
Summary:        Development files for %{name}
Requires:       %{name}%{?_isa} = %{version}-%{release}

%description    devel
The %{name}-devel package contains libraries and header files for
developing applications that use %{name}.


%prep
%setup -q -n %{name}-%{commit}
%patch0 -p1

%build
%cmake .
make %{?_smp_mflags}
doxygen

%install
rm -rf $RPM_BUILD_ROOT
make install DESTDIR=$RPM_BUILD_ROOT

%check
#ctest

%post -p /sbin/ldconfig

%postun -p /sbin/ldconfig


%files
%doc LICENSE AUTHORS
%{_libdir}/*.so.*
# For arch-specific packages: sitearch
%{python_sitearch}/*

%files devel
%doc doc/*
%{_includedir}/*
%{_libdir}/*.so


%changelog
* Sat Mar 01 2014 Dean Brettle <dean@brettle.com> - 2.0-0.3.20140228gite138769
- Changed SO_VERSION to SOVERSION so that libs only use major version number and
  added rpm directory with spec file and RPM building instructions.

* Thu Feb 27 2014 Dean Brettle <dean@brettle.com> - 2.0-0.2.20140223gite138769
- Moved shared lib version spec to top-level CMakeLists.txt

* Sun Feb 23 2014 Dean Brettle <dean@brettle.com> - 2.0-0.1.20140223gite138769
- Initial release
