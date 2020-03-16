"use strict";

import React from 'react';
import ProjectService from '../services/ProjectService';
import NewProject from "../components/NewProject";


export class NewProjectView extends React.Component {

    constructor(props) {
        super(props);
        this.state = {};
    }

    createProject(project) {
        ProjectService.create(project).then((data) => {
            this.props.history.push("/");
        }).catch((e) => {
            console.error(e);
            this.setState({
                error: e
            });
        });
    }

    render() {
        return (
            <NewProject onProjectSubmit={(project) => this.createProject(project)} error={this.state.error}/>
        );
    }
}
