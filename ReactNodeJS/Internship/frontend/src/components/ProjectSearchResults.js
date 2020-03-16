"use strict";

import React from 'react';
import { Card } from 'react-md';
import { withRouter } from 'react-router-dom';

import 'react-md/dist/react-md.indigo-pink.min.css'

class ProjectSearchResults extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
        };
    }

    render() {
        const ResultsGrid = this.props.results.map((project) =>
            <div className="col-md-4" style={{height: 145+'px'}} onClick={() => this.props.history.push(`/project/${project._id}`)}>
                <Card>
                    <div style={{height:125+'px', padding: 25+'px'}}>
                        <p className="projectSearchCardText">{project._source.title}</p>
                    </div>
                </Card>
            </div>
        );
        return (
            <div style={{paddingTop: 25 + 'px'}}>
                <div className="col-md-12" style={{paddingBottom: 50 + 'px'}}>
                    <div className="col-md-4"/>
                    <div className="col-md-4">
                        <Card className="counterBox" style={{paddingTop: 5+'px', paddingBottom: 5+'px'}}>
                            <div className="col-md-6">
                                <p className="counterBoxText">{this.props.projectCount}</p>
                                <h4 className="text-center">Projects</h4>
                            </div>
                            <div className="col-md-6 borderBoundary">
                                <p className="counterBoxText">{this.props.userCount}</p>
                                <h4 className="text-center">Retired Professionals</h4>
                            </div>
                        </Card>
                    </div>
                    <div className="col-md-4"/>
                </div>
                {ResultsGrid}
            </div>
        );
    }
};

export default withRouter(ProjectSearchResults);