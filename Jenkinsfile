#!groovy

// Jenkinsfile for compiling, testing, and packaging the Enki libraries.
// Requires CMake plugin from https://github.com/davidjsherman/aseba-jenkins.git in global library.

pipeline {
	agent any // use any available Jenkins agent

	// Trigger the build
	triggers {
		// Poll GitHub every two hours, in case webhooks aren't used
		pollSCM('H */2 * * *')
	}

	// Everything will be built in the build/ directory.
	// Everything will be installed in the dist/ directory.
	stages {
		stage('Prepare') {
			steps {
				checkout scm
			}
		}
		stage('Compile') {
			parallel {
				stage("Compile on debian") {
					agent {
						label 'debian'
					}
					steps {
						script {
							env.debian_python = sh ( script: '''
									python -c "import sys; print 'lib/python'+str(sys.version_info[0])+'.'+str(sys.version_info[1])+'/dist-packages'"
''', returnStdout: true).trim()
						}
						CMake([label: 'debian',
							   getCmakeArgs: "-DPYTHON_CUSTOM_TARGET:PATH=${env.debian_python}"])
						stash includes: 'dist/**', name: 'dist-debian'
						stash includes: 'build/**', name: 'build-debian'
					}
				}
				stage("Compile on macos") {
					agent {
						label 'macos'
					}
					steps {
						CMake([label: 'macos',
							   getCmakeArgs: "-DCMAKE_PREFIX_PATH=/usr/local/opt/qt5"])
						stash includes: 'dist/**', name: 'dist-macos'
					}
				}
				stage("Compile on windows") {
					agent {
						label 'windows-qt5'
					}
					steps {
						CMake([label: 'windows'])
						stash includes: 'dist/**', name: 'dist-windows'
					}
				}
			}
		}
		stage('Test') {
			parallel {
				stage("Test on debian") {
					agent {
						label 'debian'
					}
					steps {
						unstash 'build-debian'
						dir('build/debian') {
							sh 'LANG=C ctest'
						}
					}
				}
			}
		}
		stage('Package') {
			// Packages are only built for the master branch
			when {
				expression {
					return env.BRANCH == 'master'
				}
			}
			parallel {
				stage("Build debian package") {
					agent {
						label 'debian'
					}
					steps {
						dir('build/debian/package') {
							// We must rebuild in a subdirectory to prevent debuild from polluting the workspace parent
							sh 'git clone --depth 1 --single-branch $GIT_URL'
							sh '(cd enki && which debuild && debuild -i -us -uc -b)'
							sh 'mv libenki*.deb libenki*.changes libenki*.build $WORKSPACE/dist/debian/'
						}
						stash includes: 'dist/**', name: 'dist-debian'
					}
				}
			}
		}
		stage('Archive') {
			steps {
				unstash 'dist-debian'
				unstash 'dist-macos'
				unstash 'dist-windows'
				archiveArtifacts artifacts: 'dist/**', fingerprint: true, onlyIfSuccessful: true
			}
		}
	}
}
