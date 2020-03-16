"use strict";

import React from 'react';
import {Button, Card} from 'react-md';
import { withRouter } from 'react-router-dom';

import 'react-md/dist/react-md.indigo-pink.min.css'
import Page from "./Page";


class ProjectsList extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
        };
    }

    render() {
        var count = 0 ;
        const ResultsGrid = this.props.results.map((project) =>
            <div className="md-block-centered col-md-3" style={{height: '600px',  display:'block'}} onClick={() => this.props.history.push(`/project/${project._id}`)}>
                <Card className="md-block-centered" style={{height: '200px',  display:'block'}}>
                    <div className= "text-center" style={{ fontWeight: 'bold', padding:'30'}} >
                        <h3 style = {{padding:"1em", color:"grey" }}>Project No: {count = count+1}</h3>
                        <h2 style = {{padding:"1em" }}>{project._source.title}</h2>
                    </div>
                </Card>
            </div>
        );
        return (
            <Page>
            <div className="text-center col-md-12" style={{paddingTop:"5em"}}>
                <div className="text-center col-md-12" style={{paddingBottom: 50 + 'px'}}>
                    <h2> These are the projects that you have registered with Us.</h2>
                    <h3>Click on a project to see the detail</h3>
                    <div className="col-md-4"/>
                    <div className="col-md-4">
                    </div>
                    <div className="col-md-4"/>
                </div>
                {ResultsGrid}
            </div>
        </Page>
        );
    }
};

export default withRouter(ProjectsList);
