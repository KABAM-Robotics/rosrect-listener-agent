pipeline{
  agent { label 'ubuntu' }
  stages{
    stage('--test--'){
      steps{
        echo 'conducting tests'
      }
    }
    stage('--build--'){
      steps{
            sh '''
            ls
            sh test.sh
            '''

      }
    }
    stage ("--Extract test results--") {
    steps {
    cobertura coberturaReportFile: 'coverage.xml'
    }
    }
  }
  post {
  always {
      junit 'coverage.xml'
      step([$class: 'CoberturaPublisher', autoUpdateHealth: false, autoUpdateStability: false, coberturaReportFile: '**/coverage.xml', failUnhealthy: false, failUnstable: false, maxNumberOfBuilds: 0, onlyStable: false, sourceEncoding: 'ASCII', zoomCoverageChart: false])
  }
}

}
