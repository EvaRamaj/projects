"use strict";

import React from 'react';
import { Card, Button, FontIcon, TextField } from 'react-md';
import { withRouter } from 'react-router-dom'

import { AlertMessage } from './AlertMessage';
import Page from './Page';
import axios from 'axios';


const style = { maxWidth: 500 };


class BecomeLessorForm extends React.Component {

    constructor(props) {
        console.log(props);
        super(props);
        this.state = {
            user: props.user,
            fileNames: [],
            error: undefined,
            selectedFile0: undefined
        };

        this.fileNames = [];
        this.handleChangeFile0 = this.handleChangeFile0.bind(this);
        this.uploadHandler0 = this.uploadHandler0.bind(this);

        this.handleSubmit = this.handleSubmit.bind(this);
    }
    handleChangeFile0 (event){
        this.setState(Object.assign({}, this.state, {selectedFile0: event.target.files[0]}));
    };
    uploadHandler0(){
    };

    handleSubmit(event) {
        event.preventDefault();
        let user = this.props.user;
        if(user === undefined) {
            user = {};
        }
        if(this.state.selectedFile0 !== undefined) {
            const formData = new FormData();
            formData.append('file', this.state.selectedFile0);
            axios.post('http://localhost:3000/files', formData).then(
                response => {
                    this.fileNames.push(response.data.file.filename);
                    this.setState(Object.assign({}, this.state, {fileNames: this.fileNames}));
                    user.id_document = this.fileNames;
                    user.lessor_role_requested = true;
                    this.props.onSubmit(user);
                }
            )
        }
    }

// Usage!
    render() {
        return (
            <Page>
                <h2>Hey! Let's offer your assets!!</h2>
                <Card style={style} className="md-block-centered">
                    <input type="file" required={true} onChange={this.handleChangeFile0}/>
                    <form className="md-grid" onSubmit={this.handleSubmit} onReset={() => this.props.history.goBack()}>
                        <Button id="submit" type="submit"
                                raised primary className="md-cell md-cell--2">Upload</Button>
                        <AlertMessage className="md-row md-full-width" >{this.props.error ? `${this.props.error}` : ''}</AlertMessage>
                    </form>
                </Card>
            </Page>
        );
    }
}

export default withRouter(BecomeLessorForm);